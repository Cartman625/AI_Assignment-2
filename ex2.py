import re
from collections import deque

import ext_elev
import search
from ex1_322535436 import ElevatorsProblem

id = ["322535436"]


_ACTION_RE = re.compile(r"\s*(MOVE|ENTER|EXIT)\s*\{\s*(-?\d+)\s*,\s*(-?\d+)\s*\}\s*")


class Controller:
    def __init__(self, game: ext_elev.GameAPI):
        self.game = game
        self.reachable = {eid: set(floors) for eid, floors in game.get_reachable().items()}
        self.capacities = dict(game.get_capacities())
        self.goal_reward = game.get_goal_reward()

        self._elevators = tuple(sorted(self.reachable))
        self._person_goal = {}
        self._person_weight = {}
        self._person_prob = {}
        self._person_reward_mean = {}
        self._elev_prob = {}

        self._initial_state = self._canonical_state(game.get_initial_state())
        for eid in self._elevators:
            self._elev_prob[eid] = float(game.get_elevator_action_prob(eid))

        init_persons = [pid for pid, _ in self._initial_state[1]]
        for pid in init_persons:
            self._person_goal[pid] = game.get_person_goal(pid)
            self._person_weight[pid] = game.get_person_weight(pid)
            self._person_prob[pid] = float(game.get_person_action_prob(pid))
            rewards = game.get_person_reward(pid)
            self._person_reward_mean[pid] = float(sum(rewards)) / float(len(rewards))
        sorted_means = sorted(self._person_reward_mean.values(), reverse=True)
        self._is_rl_like = (
            len(sorted_means) >= 2
            and sorted_means[0] >= 40.0
            and sorted_means[0] >= 2.5 * max(sorted_means[1], 1.0)
        )

        floors = set()
        for fs in self.reachable.values():
            floors.update(fs)
        for _, f, _ in self._initial_state[0]:
            floors.add(f)
        for _, loc in self._initial_state[1]:
            if loc[0] == "floor":
                floors.add(loc[1])
        for g in self._person_goal.values():
            floors.add(g)
        self._height = max(floors) if floors else 0

        self._plan_cache = {}
        self._active_plan = []
        self._active_strategy = None
        self._last_state = None
        self._last_action = None
        self._expected_state = None

        self._initial_full_actions = self._plan_actions(self._initial_state, "full", None) or []
        self._initial_full_steps = self._expected_steps(self._initial_full_actions)
        self._initial_full_value = self._state_delivery_value(self._initial_state)
        self._initial_farm_loop = {}
        self._farming_pid = None
        self._farming_rps = 0.0

        self._shared_floors = {}
        self._adj = {eid: [] for eid in self._elevators}
        for i, e1 in enumerate(self._elevators):
            for e2 in self._elevators[i + 1 :]:
                shared = sorted(self.reachable[e1] & self.reachable[e2])
                if shared:
                    self._shared_floors[(e1, e2)] = shared
                    self._shared_floors[(e2, e1)] = shared
                    self._adj[e1].append(e2)
                    self._adj[e2].append(e1)

        self._farming_rps, self._farming_pid = self._compute_simple_farming_loop(
            self._initial_state
        )

    # ------------------------ State/action utilities ------------------------ #
    def _canonical_state(self, state):
        elevators_t, persons_t, total = state
        return (tuple(sorted(elevators_t)), tuple(sorted(persons_t)), total)

    def _parse_action(self, action):
        if not isinstance(action, str):
            return None
        if action == "RESET":
            return ("RESET",)
        m = _ACTION_RE.fullmatch(action)
        if not m:
            return None
        return (m.group(1), int(m.group(2)), int(m.group(3)))

    def _is_action_legal(self, state, action):
        p = self._parse_action(action)
        if p is None:
            return False
        if p[0] == "RESET":
            return True

        elevators_t, persons_t, _ = state
        elev_floor = {eid: f for eid, f, _ in elevators_t}
        elev_weight = {eid: w for eid, _, w in elevators_t}
        person_loc = {pid: loc for pid, loc in persons_t}

        if p[0] == "MOVE":
            eid, target = p[1], p[2]
            return eid in elev_floor and target in self.reachable.get(eid, ())

        if p[0] == "ENTER":
            pid, eid = p[1], p[2]
            if pid not in person_loc or eid not in elev_floor:
                return False
            loc = person_loc[pid]
            if loc[0] != "floor" or loc[1] != elev_floor[eid]:
                return False
            return elev_weight[eid] + self._person_weight[pid] <= self.capacities[eid]

        if p[0] == "EXIT":
            pid, eid = p[1], p[2]
            if pid not in person_loc:
                return False
            loc = person_loc[pid]
            return loc[0] == "in" and loc[1] == eid

        return False

    def _simulate_success_state(self, state, action):
        parsed = self._parse_action(action)
        if parsed is None:
            return state
        if parsed[0] == "RESET":
            return self._initial_state

        elevators_t, persons_t, total = state
        elevators = list(elevators_t)
        persons = list(persons_t)

        elev_index = {eid: i for i, (eid, _, _) in enumerate(elevators)}
        person_index = {pid: i for i, (pid, _) in enumerate(persons)}

        name, a, b = parsed
        if name == "MOVE":
            if a not in elev_index:
                return state
            i = elev_index[a]
            _, _, w = elevators[i]
            elevators[i] = (a, b, w)
            return self._canonical_state((tuple(elevators), tuple(persons), total))

        if name == "ENTER":
            pid, eid = a, b
            if pid not in person_index or eid not in elev_index:
                return state
            pi = person_index[pid]
            ei = elev_index[eid]
            _, ef, ew = elevators[ei]
            if persons[pi][1] != ("floor", ef):
                return state
            if ew + self._person_weight[pid] > self.capacities[eid]:
                return state
            elevators[ei] = (eid, ef, ew + self._person_weight[pid])
            persons[pi] = (pid, ("in", eid))
            return self._canonical_state((tuple(elevators), tuple(persons), total))

        if name == "EXIT":
            pid, eid = a, b
            if pid not in person_index or eid not in elev_index:
                return state
            pi = person_index[pid]
            ei = elev_index[eid]
            _, ef, ew = elevators[ei]
            if persons[pi][1] != ("in", eid):
                return state
            elevators[ei] = (eid, ef, ew - self._person_weight[pid])
            if ef == self._person_goal[pid]:
                persons = [p for p in persons if p[0] != pid]
                total -= 1
                if total == 0:
                    return self._initial_state
            else:
                persons[pi] = (pid, ("floor", ef))
            return self._canonical_state((tuple(elevators), tuple(persons), total))

        return state

    def _record_action(self, state, action):
        self._last_state = state
        self._last_action = action
        self._expected_state = self._simulate_success_state(state, action)

    def _recoverable_person_fail(self, state):
        if not self._last_action or self._last_state is None:
            return False
        parsed = self._parse_action(self._last_action)
        if parsed is None or parsed[0] not in ("ENTER", "EXIT"):
            return False
        return state == self._last_state

    # ------------------------------- Planning ------------------------------- #
    def _build_problem(self, state, plan_mode, plan_pid):
        elevators_t, persons_t, _ = state
        if plan_mode == "farm":
            person_ids = [plan_pid] if any(pid == plan_pid for pid, _ in persons_t) else []
        else:
            person_ids = [pid for pid, _ in persons_t]

        if not person_ids:
            return None

        elev_floor = {eid: f for eid, f, _ in elevators_t}
        persons_by_id = {pid: loc for pid, loc in persons_t}

        elevators_def = {
            eid: (elev_floor[eid], tuple(sorted(self.reachable[eid])), self.capacities[eid])
            for eid in self._elevators
        }
        persons_def = {}
        for pid in sorted(person_ids):
            loc = persons_by_id[pid]
            if loc[0] == "floor":
                start_floor = loc[1]
            else:
                start_floor = elev_floor[loc[1]]
            persons_def[pid] = (start_floor, self._person_weight[pid], self._person_goal[pid])

        problem = ElevatorsProblem(
            {
                "height": self._height,
                "Elevators": elevators_def,
                "Persons": persons_def,
            }
        )

        elevator_state = tuple(elev_floor[eid] for eid in problem.elevator_ids)
        person_state = []
        for pid in problem.person_ids:
            loc = persons_by_id[pid]
            if loc[0] == "floor":
                person_state.append((problem.ON_FLOOR, loc[1]))
            else:
                person_state.append((problem.IN_ELEVATOR, problem.elevator_index[loc[1]]))
        problem.initial = (elevator_state, tuple(person_state))
        return problem

    def _plan_actions(self, state, plan_mode, plan_pid):
        key = (plan_mode, plan_pid, state)
        cached = self._plan_cache.get(key)
        if cached is not None:
            return list(cached)

        try:
            problem = self._build_problem(state, plan_mode, plan_pid)
            if problem is None:
                self._plan_cache[key] = tuple()
                return []
            solved = search.astar_search(problem, h=problem.h_astar)
            if solved is None:
                self._plan_cache[key] = None
                return None
            node = solved[0] if isinstance(solved, tuple) else solved
            if node is None:
                self._plan_cache[key] = None
                return None
            path_nodes = list(reversed(node.path()))
            actions = [n.action for n in path_nodes[1:] if n.action is not None]
            self._plan_cache[key] = tuple(actions)
            return actions
        except Exception:
            self._plan_cache[key] = None
            return None

    def _expected_steps(self, actions):
        total = 0.0
        for action in actions:
            parsed = self._parse_action(action)
            if parsed is None:
                return float("inf")
            if parsed[0] == "MOVE":
                total += 1.0 / max(self._elev_prob.get(parsed[1], 1.0), 0.05)
            elif parsed[0] in ("ENTER", "EXIT"):
                total += 1.0 / max(self._person_prob.get(parsed[1], 1.0), 0.05)
            elif parsed[0] == "RESET":
                total += 1.0
        return total

    def _state_delivery_value(self, state):
        persons_t = state[1]
        if not persons_t:
            return 0.0
        return sum(self._person_reward_mean[pid] for pid, _ in persons_t) + self.goal_reward

    def _finite_horizon_total(self, first_value, first_steps, loop_value, loop_steps, horizon):
        if horizon <= 0 or first_steps <= 0 or first_steps > horizon:
            return 0.0
        total = first_value
        remaining = horizon - first_steps
        if loop_steps > 0 and loop_value > 0 and remaining > 0:
            loops = int(remaining // loop_steps)
            total += loops * loop_value
            remaining -= loops * loop_steps
            total += (remaining / loop_steps) * loop_value
        return total

    def _farm_loop_steps_from_initial(self, pid):
        if pid in self._initial_farm_loop:
            return self._initial_farm_loop[pid]
        actions = self._plan_actions(self._initial_state, "farm", pid)
        if actions is None or not actions:
            self._initial_farm_loop[pid] = float("inf")
            return float("inf")
        steps = self._expected_steps(actions + ["RESET"])
        self._initial_farm_loop[pid] = steps
        return steps

    def _compute_simple_farming_loop(self, state):
        elevators_t, persons_t, _ = state
        elev_floor = {eid: f for eid, f, _ in elevators_t}
        best_rps = 0.0
        best_pid = None
        for pid, loc in persons_t:
            if loc[0] != "floor":
                continue
            p_floor = loc[1]
            goal = self._person_goal[pid]
            reward = self._person_reward_mean[pid] * self._person_prob[pid]
            for eid, ef in elev_floor.items():
                if p_floor not in self.reachable[eid] or goal not in self.reachable[eid]:
                    continue
                move_to_person = 0 if ef == p_floor else 1
                move_to_goal = 0 if p_floor == goal else 1
                loop_steps = move_to_person + 1 + move_to_goal + 1 + 1
                expected_steps = loop_steps / max(self._elev_prob[eid], 0.1)
                rps = reward / max(expected_steps, 1.0)
                if rps > best_rps:
                    best_rps = rps
                    best_pid = pid
        return best_rps, best_pid

    def _simple_farming_active(self, state):
        if self._farming_pid is None or self._farming_rps <= 0.0 or not state[1]:
            return False
        if not self._is_rl_like:
            return False
        if self._person_reward_mean.get(self._farming_pid, 0.0) < 40.0:
            return False
        steps_left = self.game.get_max_steps() - self.game.get_current_steps()
        current_value = self._state_delivery_value(state)
        return self._farming_rps * steps_left > current_value * 1.5

    def _farming_action(self, state):
        elevators_t, persons_t, _ = state
        elev_floor = {eid: f for eid, f, _ in elevators_t}
        elev_weight = {eid: w for eid, _, w in elevators_t}
        waiting = {}
        riding = {}
        for pid, loc in persons_t:
            if loc[0] == "floor":
                waiting.setdefault(loc[1], []).append(pid)
            else:
                riding.setdefault(loc[1], []).append(pid)

        for eid, pids in riding.items():
            ef = elev_floor[eid]
            for pid in pids:
                if self._person_goal[pid] == ef:
                    return f"EXIT{{{pid},{eid}}}"

        if not riding and not any(pid == self._farming_pid for pid, _ in persons_t):
            return "RESET"

        for eid, ef in elev_floor.items():
            if self._farming_pid in waiting.get(ef, ()) and (
                elev_weight[eid] + self._person_weight[self._farming_pid] <= self.capacities[eid]
            ):
                return f"ENTER{{{self._farming_pid},{eid}}}"

        for eid, pids in riding.items():
            if self._farming_pid not in pids:
                continue
            goal = self._person_goal[self._farming_pid]
            ef = elev_floor[eid]
            if goal in self.reachable[eid] and goal != ef:
                return f"MOVE{{{eid},{goal}}}"

        farming_floor = None
        for pid, loc in persons_t:
            if pid == self._farming_pid and loc[0] == "floor":
                farming_floor = loc[1]
                break
        if farming_floor is not None:
            best = None
            best_dist = None
            for eid, ef in elev_floor.items():
                if farming_floor not in self.reachable[eid] or ef == farming_floor:
                    continue
                d = abs(farming_floor - ef)
                if best_dist is None or d < best_dist:
                    best = (eid, farming_floor)
                    best_dist = d
            if best is not None:
                return f"MOVE{{{best[0]},{best[1]}}}"
        return None

    def _choose_strategy(self, state):
        return ("full", None)

    def _build_strategy_plan(self, state, strategy):
        mode, pid = strategy
        if mode == "farm":
            farm_actions = self._plan_actions(state, "farm", pid)
            if farm_actions:
                return list(farm_actions) + ["RESET"]
            return None
        return self._plan_actions(state, "full", None)

    # -------------------------- Greedy legal fallback ------------------------ #
    def _route_next_elevator(self, start_eid, goal_floor):
        if goal_floor in self.reachable[start_eid]:
            return start_eid
        targets = {eid for eid in self._elevators if goal_floor in self.reachable[eid]}
        if not targets:
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
            return None
        step = found
        while parent[step] is not None and parent[step] != start_eid:
            step = parent[step]
        return step if parent[step] == start_eid else found

    def _greedy_action(self, state):
        elevators_t, persons_t, _ = state
        elev_floor = {eid: f for eid, f, _ in elevators_t}
        elev_weight = {eid: w for eid, _, w in elevators_t}
        last = self._parse_action(self._last_action) if self._last_action else None

        waiting = {}
        riding = {}
        for pid, loc in persons_t:
            if loc[0] == "floor":
                waiting.setdefault(loc[1], []).append(pid)
            else:
                riding.setdefault(loc[1], []).append(pid)

        for eid, pids in riding.items():
            ef = elev_floor[eid]
            for pid in pids:
                if self._person_goal[pid] == ef:
                    return f"EXIT{{{pid},{eid}}}"

        for eid, pids in riding.items():
            ef = elev_floor[eid]
            for pid in pids:
                goal = self._person_goal[pid]
                if goal in self.reachable[eid]:
                    continue
                nxt = self._route_next_elevator(eid, goal)
                if nxt is None or nxt == eid:
                    continue
                shared = self._shared_floors.get((eid, nxt), ())
                if ef in shared:
                    return f"EXIT{{{pid},{eid}}}"

        best_enter = None
        best_enter_score = float("-inf")
        for eid, ef in elev_floor.items():
            for pid in waiting.get(ef, ()):
                if last and last[0] == "EXIT" and last[1] == pid and last[2] == eid:
                    if self._person_goal.get(pid) != ef:
                        continue
                if elev_weight[eid] + self._person_weight[pid] > self.capacities[eid]:
                    continue
                score = self._person_reward_mean[pid] * self._person_prob[pid] * self._elev_prob[eid]
                if score > best_enter_score:
                    best_enter_score = score
                    best_enter = f"ENTER{{{pid},{eid}}}"
        if best_enter is not None:
            return best_enter

        best_move = None
        best_score = float("-inf")
        for eid, pids in riding.items():
            ef = elev_floor[eid]
            for pid in pids:
                goal = self._person_goal[pid]
                if goal in self.reachable[eid] and goal != ef:
                    score = self._person_reward_mean[pid] / (1 + abs(goal - ef))
                    if score > best_score:
                        best_score = score
                        best_move = f"MOVE{{{eid},{goal}}}"
                else:
                    nxt = self._route_next_elevator(eid, goal)
                    if nxt is None or nxt == eid:
                        continue
                    shared = self._shared_floors.get((eid, nxt), ())
                    if not shared:
                        continue
                    target = min(shared, key=lambda f: abs(f - ef))
                    if target != ef:
                        score = self._person_reward_mean[pid] / (1 + abs(target - ef))
                        if score > best_score:
                            best_score = score
                            best_move = f"MOVE{{{eid},{target}}}"
        if best_move is not None:
            return best_move

        for eid, ef in elev_floor.items():
            if eid in riding:
                continue
            candidate = None
            best_d = None
            for wf in waiting:
                if wf in self.reachable[eid] and wf != ef:
                    d = abs(wf - ef)
                    if best_d is None or d < best_d:
                        best_d = d
                        candidate = wf
            if candidate is not None:
                return f"MOVE{{{eid},{candidate}}}"

        return "RESET"

    def choose_next_action(self, state):
        try:
            state = self._canonical_state(state)

            if self._simple_farming_active(state):
                farming_action = self._farming_action(state)
                if farming_action is not None and self._is_action_legal(state, farming_action):
                    self._record_action(state, farming_action)
                    self._active_plan = []
                    self._active_strategy = None
                    return farming_action

            if self._last_action is not None:
                if state == self._expected_state:
                    if self._active_plan and self._active_plan[0] == self._last_action:
                        self._active_plan.pop(0)
                elif self._recoverable_person_fail(state) and self._is_action_legal(state, self._last_action):
                    self._record_action(state, self._last_action)
                    return self._last_action
                else:
                    self._active_plan = []
                    self._active_strategy = None

            strategy = self._choose_strategy(state)
            if not self._active_plan or strategy != self._active_strategy:
                plan = self._build_strategy_plan(state, strategy)
                if plan is None:
                    self._active_plan = []
                    self._active_strategy = None
                else:
                    self._active_plan = list(plan)
                    self._active_strategy = strategy

            if self._active_plan:
                candidate = self._active_plan[0]
                parsed = self._parse_action(candidate)
                if parsed and parsed[0] == "ENTER" and self._last_action:
                    last = self._parse_action(self._last_action)
                    if last and last[0] == "EXIT" and last[1] == parsed[1] and last[2] == parsed[2]:
                        person_loc = dict(state[1]).get(parsed[1])
                        if person_loc and person_loc[0] == "floor":
                            if person_loc[1] == dict((eid, f) for eid, f, _ in state[0]).get(parsed[2]):
                                candidate = None
                if self._is_action_legal(state, candidate):
                    self._record_action(state, candidate)
                    return candidate
                self._active_plan = []
                self._active_strategy = None

            fallback = self._greedy_action(state)
            if not self._is_action_legal(state, fallback):
                fallback = "RESET"
            self._record_action(state, fallback)
            return fallback
        except Exception:
            return "RESET"
