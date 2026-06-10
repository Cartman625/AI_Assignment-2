import ext_elev
from collections import deque

id = ["322535436"]


class Controller:
    """Greedy stochastic multi-elevator controller.

    Action priority each step:
      1. EXIT a passenger who is at their goal floor.
      1b. EXIT for transfer (elevator cannot reach passenger's goal).
      2. ENTER a waiting person into an elevator on the same floor.
      3. MOVE a loaded elevator toward its passengers' nearest goal.
      4. Reset-farm: RESET when looping a cheap high-reward delivery beats
         full delivery of low-value remaining persons.
      5. MOVE an empty elevator toward the nearest waiting person.
    """

    def __init__(self, game: ext_elev.GameAPI):
        self.game = game
        self.reachable = game.get_reachable()
        self.capacities = game.get_capacities()
        self.goal_reward = game.get_goal_reward()
        # Cache static person/elevator attributes.
        self._goal_cache = {}
        self._weight_cache = {}
        self._reward_mean_cache = {}
        self._person_prob_cache = {}
        self._elev_prob_cache = {}

        self._elevators = sorted(self.reachable.keys())
        self._shared_floors = {}
        self._adj = {eid: [] for eid in self._elevators}
        for i, e1 in enumerate(self._elevators):
            for e2 in self._elevators[i + 1:]:
                shared = sorted(self.reachable[e1] & self.reachable[e2])
                if not shared:
                    continue
                self._shared_floors[(e1, e2)] = shared
                self._shared_floors[(e2, e1)] = shared
                self._adj[e1].append(e2)
                self._adj[e2].append(e1)

        self._route_cache = {}

        # Precompute reset-farming metrics from the initial state.
        init_elev_t, init_persons_t, _ = game.get_initial_state()
        init_elev_floor = {eid: f for eid, f, _ in init_elev_t}
        self._farming_rps, self._farming_pid = self._compute_loop_rps(
            init_persons_t, init_elev_floor
        )
        full_init_val = (
            sum(self._reward_mean(pid) for pid, _ in init_persons_t) + self.goal_reward
        )
        # Break-even: minimum steps_remaining for farming to beat full delivery.
        # farming_rps * steps > full_val * 1.5  →  steps > full_val * 1.5 / farming_rps
        if self._farming_rps > 0:
            self._farming_breakeven = (full_init_val * 1.5) / self._farming_rps
        else:
            self._farming_breakeven = float("inf")

    # ------------------------------------------------------------------ #
    # Helpers                                                             #
    # ------------------------------------------------------------------ #
    def _goal(self, pid):
        if pid not in self._goal_cache:
            self._goal_cache[pid] = self.game.get_person_goal(pid)
        return self._goal_cache[pid]

    def _weight(self, pid):
        if pid not in self._weight_cache:
            self._weight_cache[pid] = self.game.get_person_weight(pid)
        return self._weight_cache[pid]

    def _reward_mean(self, pid):
        if pid not in self._reward_mean_cache:
            rewards = self.game.get_person_reward(pid)
            self._reward_mean_cache[pid] = float(sum(rewards)) / float(len(rewards))
        return self._reward_mean_cache[pid]

    def _person_prob(self, pid):
        if pid not in self._person_prob_cache:
            self._person_prob_cache[pid] = self.game.get_person_action_prob(pid)
        return self._person_prob_cache[pid]

    def _elev_prob(self, eid):
        """Return elevator action success probability (cached)."""
        if eid not in self._elev_prob_cache:
            self._elev_prob_cache[eid] = self.game.get_elevator_action_prob(eid)
        return self._elev_prob_cache[eid]

    def _closest_reachable(self, eid, cur_floor, target_floor):
        """Return the reachable floor closest to *target_floor*, excluding
        the elevator's current floor.  Returns None if no move exists."""
        r = self.reachable[eid]
        if target_floor in r and target_floor != cur_floor:
            return target_floor
        candidates = [f for f in r if f != cur_floor]
        if not candidates:
            return None
        return min(candidates, key=lambda f: abs(f - target_floor))

    def _route_next_elevator(self, start_eid, goal_floor):
        """Best next elevator on a shortest transfer path from start_eid to an
        elevator that can reach goal_floor. Returns None if impossible."""
        key = (start_eid, goal_floor)
        if key in self._route_cache:
            return self._route_cache[key]

        if goal_floor in self.reachable[start_eid]:
            self._route_cache[key] = start_eid
            return start_eid

        targets = {eid for eid in self._elevators if goal_floor in self.reachable[eid]}
        if not targets:
            self._route_cache[key] = None
            return None

        q = deque([start_eid])
        parent = {start_eid: None}
        found = None
        while q:
            cur = q.popleft()
            if cur in targets:
                found = cur
                break
            for nxt in self._adj[cur]:
                if nxt not in parent:
                    parent[nxt] = cur
                    q.append(nxt)

        if found is None:
            self._route_cache[key] = None
            return None

        step = found
        while parent[step] is not None and parent[step] != start_eid:
            step = parent[step]
        next_elevator = step if parent[step] == start_eid else found
        self._route_cache[key] = next_elevator
        return next_elevator

    def _best_transfer_floor(self, from_eid, to_eid, cur_floor, elev_floor=None):
        shared = self._shared_floors.get((from_eid, to_eid), ())
        if not shared:
            return None
        if elev_floor is not None:
            to_pos = elev_floor.get(to_eid)
            if to_pos is not None:
                # Prefer the shared floor closest to where the target elevator is,
                # so the person is dropped off where the handoff is most likely.
                return min(shared, key=lambda f: abs(f - to_pos))
        return min(shared, key=lambda f: abs(f - cur_floor))

    def _compute_loop_rps(self, persons_t, elev_floor):
        """Compute best reward-per-step for a reset-farming loop.

        A loop = deliver one person from their initial floor to goal + RESET.
        Returns (best_rps, best_pid).
        """
        best_rps = 0.0
        best_pid = None
        for pid, loc in persons_t:
            if loc[0] != "floor":
                continue
            p_floor = loc[1]
            goal = self._goal(pid)
            for eid, ef in elev_floor.items():
                if p_floor not in self.reachable[eid]:
                    continue
                if goal not in self.reachable[eid]:
                    continue
                # Steps per loop (steady-state, starting from initial floor):
                #   move to person's floor (0 if already there) +
                #   ENTER + move to goal (0 if same floor) + EXIT + RESET
                move_to_person = 0 if ef == p_floor else 1
                move_to_goal = 0 if p_floor == goal else 1
                loop_steps = move_to_person + 1 + move_to_goal + 1 + 1
                # Inflate by elevator unreliability (expected moves per success).
                ep = self._elev_prob(eid)
                effective_steps = loop_steps / max(ep, 0.1)
                reward = self._reward_mean(pid) * self._person_prob(pid)
                rps = reward / max(effective_steps, 1)
                if rps > best_rps:
                    best_rps = rps
                    best_pid = pid
        return best_rps, best_pid

    # ------------------------------------------------------------------ #
    # Main decision                                                       #
    # ------------------------------------------------------------------ #
    def choose_next_action(self, state):
        """Return one of: "MOVE{e,f}", "ENTER{p,e}", "EXIT{p,e}", "RESET"."""
        elevators_t, persons_t, _ = state

        elev_floor = {eid: f for eid, f, _ in elevators_t}
        elev_weight = {eid: w for eid, _, w in elevators_t}

        # Determine whether reset-farming is currently more valuable than
        # completing full delivery of all remaining persons.
        steps_remaining = self.game.get_max_steps() - self.game.get_current_steps()
        if self._farming_pid is not None and self._farming_rps > 0 and persons_t:
            # Dynamic check: compare expected loop value vs current remaining value.
            current_delivery_value = (
                sum(self._reward_mean(pid) for pid, _ in persons_t) + self.goal_reward
            )
            farming_active = (
                self._farming_rps * steps_remaining > current_delivery_value * 1.5
            )
        else:
            farming_active = False

        # Classify persons into waiting-on-floor and riding-in-elevator.
        waiting = {}   # floor -> [pid]
        riding = {}    # eid  -> [pid]
        for pid, loc in persons_t:
            if loc[0] == "floor":
                waiting.setdefault(loc[1], []).append(pid)
            else:                          # loc[0] == 'in'
                riding.setdefault(loc[1], []).append(pid)

        # 1. EXIT a passenger who has reached their goal floor.
        for eid, pids in riding.items():
            ef = elev_floor[eid]
            for pid in pids:
                if self._goal(pid) == ef:
                    return f"EXIT{{{pid},{eid}}}"

        # 1b. EXIT for transfer if this elevator cannot reach the passenger goal.
        for eid, pids in riding.items():
            ef = elev_floor[eid]
            for pid in pids:
                goal = self._goal(pid)
                if goal in self.reachable[eid]:
                    continue
                next_eid = self._route_next_elevator(eid, goal)
                if next_eid is None or next_eid == eid:
                    continue
                transfer_floor = self._best_transfer_floor(eid, next_eid, ef, elev_floor)
                if transfer_floor is not None and transfer_floor == ef:
                    return f"EXIT{{{pid},{eid}}}"

        # 2. ENTER a waiting person into an elevator on the same floor.
        # When farming, only board the designated farming person so we avoid
        # wasting capacity on low-value passengers.
        # Elevator reliability is factored in so broken elevators are skipped
        # when a reliable alternative exists on the same floor.
        best_enter = None
        best_enter_score = float("-inf")
        for eid, ef in elev_floor.items():
            if ef not in waiting:
                continue
            ew = elev_weight[eid]
            cap = self.capacities[eid]
            ep = self._elev_prob(eid)
            for pid in waiting[ef]:
                if farming_active and pid != self._farming_pid:
                    continue  # don't board non-farming passengers in farming mode
                if ew + self._weight(pid) <= cap:
                    goal = self._goal(pid)
                    next_eid = self._route_next_elevator(eid, goal)
                    if next_eid is None:
                        continue
                    if next_eid != eid and ef in self._shared_floors.get((eid, next_eid), ()):
                        # Person should board the next-hop elevator, not this one.
                        continue
                    score = self._reward_mean(pid) * self._person_prob(pid) * ep
                    if goal in self.reachable[eid]:
                        score += 1000.0
                    if score > best_enter_score:
                        best_enter_score = score
                        best_enter = f"ENTER{{{pid},{eid}}}"
        if best_enter is not None:
            return best_enter

        # 3. MOVE loaded elevators toward direct goals or transfer floors.
        # Elevator reliability is factored into the score so unreliable
        # elevators are given lower priority when alternatives exist.
        best_move = None
        best_move_score = float("-inf")
        for eid, pids in riding.items():
            ef = elev_floor[eid]
            ep = self._elev_prob(eid)
            chosen = None
            for pid in pids:
                goal = self._goal(pid)
                if goal in self.reachable[eid]:
                    if goal == ef:
                        continue
                    target = goal
                    score = (self._reward_mean(pid) * self._person_prob(pid) * ep) / (
                        1 + abs(goal - ef)
                    )
                    score += 100.0
                else:
                    next_eid = self._route_next_elevator(eid, goal)
                    if next_eid is None or next_eid == eid:
                        continue
                    target = self._best_transfer_floor(eid, next_eid, ef, elev_floor)
                    if target is None or target == ef:
                        continue
                    score = (self._reward_mean(pid) * self._person_prob(pid) * ep) / (
                        1 + abs(target - ef)
                    )

                if chosen is None or score > chosen[0]:
                    chosen = (score, target)

            if chosen is None:
                continue
            _, target = chosen
            nf = self._closest_reachable(eid, ef, target)
            if nf is not None:
                score = -abs(target - ef) * ep
                if score > best_move_score:
                    best_move_score = score
                    best_move = f"MOVE{{{eid},{nf}}}"
        if best_move is not None:
            return best_move

        # 4. Reset-farming: when all elevators are empty and the farming
        # person has been delivered this cycle (they are absent from persons_t),
        # RESET to restart the loop rather than slowly delivering low-value
        # remaining persons.
        if farming_active and not riding:
            farming_person_present = any(pid == self._farming_pid for pid, _ in persons_t)
            if not farming_person_present:
                return "RESET"

        # 5. MOVE an empty elevator toward the nearest waiting person.
        if not waiting:
            return "RESET"

        best_empty_move = None
        best_empty_score = float("-inf")
        for eid, ef in elev_floor.items():
            if eid in riding:
                continue
            ep = self._elev_prob(eid)

            target = None
            target_score = float("-inf")
            for wf, pids in waiting.items():
                if wf == ef:
                    # Persons at the elevator's current floor are handled by
                    # the ENTER step; skip them here to avoid choosing the
                    # current floor as the "move target" (which stalls the
                    # elevator when ENTER is blocked by the transfer filter).
                    continue
                if wf not in self.reachable[eid]:
                    continue
                for pid in pids:
                    if self._route_next_elevator(eid, self._goal(pid)) is None:
                        continue
                    score = (
                        self._reward_mean(pid) * self._person_prob(pid) * ep
                    ) / (1 + abs(wf - ef))
                    if score > target_score:
                        target_score = score
                        target = wf

            if target is None or target == ef:
                continue

            nf = self._closest_reachable(eid, ef, target)
            if nf is not None:
                if target_score > best_empty_score:
                    best_empty_score = target_score
                    best_empty_move = f"MOVE{{{eid},{nf}}}"
        if best_empty_move is not None:
            return best_empty_move

        return "RESET"
