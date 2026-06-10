import ext_elev
from collections import deque

id = ["322535436"]


class Controller:
    """Greedy stochastic multi-elevator controller.

    Action priority each step:
      1. EXIT a passenger who is at their goal floor.
      2. ENTER a waiting person into an elevator on the same floor
         (if capacity allows).
      3. MOVE a loaded elevator toward its passengers' nearest goal.
      4. MOVE an empty elevator toward the nearest waiting person.
    """

    def __init__(self, game: ext_elev.GameAPI):
        self.game = game
        self.reachable = game.get_reachable()
        self.capacities = game.get_capacities()
        self.goal_reward = game.get_goal_reward()
        # Cache static person attributes (goals and weights never change).
        self._goal_cache = {}
        self._weight_cache = {}
        self._reward_mean_cache = {}
        self._person_prob_cache = {}

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

    def _closest_reachable(self, eid, cur_floor, target_floor):
        """Return the reachable floor closest to *target_floor*, excluding
        the elevator's current floor.  Returns None if no move exists."""
        r = self.reachable[eid]
        # Direct jump preferred when the target is reachable in one step.
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

    def _best_transfer_floor(self, from_eid, to_eid, cur_floor):
        shared = self._shared_floors.get((from_eid, to_eid), ())
        if not shared:
            return None
        return min(shared, key=lambda f: abs(f - cur_floor))

    # ------------------------------------------------------------------ #
    # Main decision                                                       #
    # ------------------------------------------------------------------ #
    def choose_next_action(self, state):
        """Return one of: "MOVE{e,f}", "ENTER{p,e}", "EXIT{p,e}", "RESET"."""
        elevators_t, persons_t, _ = state

        elev_floor = {eid: f for eid, f, _ in elevators_t}
        elev_weight = {eid: w for eid, _, w in elevators_t}

        # Classify persons into waiting-on-floor and riding-in-elevator.
        waiting = {}   # floor -> [pid]
        riding = {}    # eid  -> [pid]
        for pid, loc in persons_t:
            if loc[0] == 'floor':
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
                transfer_floor = self._best_transfer_floor(eid, next_eid, ef)
                if transfer_floor is not None and transfer_floor == ef:
                    return f"EXIT{{{pid},{eid}}}"

        # 2. ENTER a waiting person into an elevator on the same floor.
        best_enter = None
        best_enter_score = float("-inf")
        for eid, ef in elev_floor.items():
            if ef not in waiting:
                continue
            ew = elev_weight[eid]
            cap = self.capacities[eid]
            for pid in waiting[ef]:
                if ew + self._weight(pid) <= cap:
                    goal = self._goal(pid)
                    next_eid = self._route_next_elevator(eid, goal)
                    if next_eid is None:
                        continue
                    if next_eid != eid and ef in self._shared_floors.get((eid, next_eid), ()):
                        # Already on a transfer floor to a better elevator.
                        continue
                    score = self._reward_mean(pid) * self._person_prob(pid)
                    if goal in self.reachable[eid]:
                        score += 1000.0
                    if score > best_enter_score:
                        best_enter_score = score
                        best_enter = f"ENTER{{{pid},{eid}}}"
        if best_enter is not None:
            return best_enter

        # 3. MOVE loaded elevators toward direct goals or transfer floors.
        best_move = None
        best_move_score = float("-inf")
        for eid, pids in riding.items():
            ef = elev_floor[eid]
            chosen = None
            for pid in pids:
                goal = self._goal(pid)
                if goal in self.reachable[eid]:
                    if goal == ef:
                        continue
                    target = goal
                    score = (self._reward_mean(pid) * self._person_prob(pid)) / (1 + abs(goal - ef))
                    score += 100.0
                else:
                    next_eid = self._route_next_elevator(eid, goal)
                    if next_eid is None or next_eid == eid:
                        continue
                    target = self._best_transfer_floor(eid, next_eid, ef)
                    if target is None or target == ef:
                        continue
                    score = (self._reward_mean(pid) * self._person_prob(pid)) / (1 + abs(target - ef))

                if chosen is None or score > chosen[0]:
                    chosen = (score, target)

            if chosen is None:
                continue
            _, target = chosen
            nf = self._closest_reachable(eid, ef, target)
            if nf is not None:
                score = -abs(target - ef)
                if score > best_move_score:
                    best_move_score = score
                    best_move = f"MOVE{{{eid},{nf}}}"
        if best_move is not None:
            return best_move

        # 4. MOVE an empty elevator toward the nearest waiting person.
        if not waiting:
            return "RESET"

        best_empty_move = None
        best_empty_score = float("-inf")
        for eid, ef in elev_floor.items():
            if eid in riding:
                continue

            target = None
            target_score = float("-inf")
            for wf, pids in waiting.items():
                if wf not in self.reachable[eid]:
                    continue
                for pid in pids:
                    if self._route_next_elevator(eid, self._goal(pid)) is None:
                        continue
                    score = (self._reward_mean(pid) * self._person_prob(pid)) / (1 + abs(wf - ef))
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
