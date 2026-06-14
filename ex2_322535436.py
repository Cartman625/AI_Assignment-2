import bisect
import re
from collections import deque

import ext_elev

id = ["322535436"]


_ACTION_RE = re.compile(r"\s*(MOVE|ENTER|EXIT)\s*\{\s*(-?\d+)\s*,\s*(-?\d+)\s*\}\s*")


infinity = 1.0e400


def update(x, **entries):
    if isinstance(x, dict):
        x.update(entries)
    else:
        x.__dict__.update(entries)
    return x


def memoize(fn, slot=None):
    if slot:
        def memoized_fn(obj, *args):
            if hasattr(obj, slot):
                return getattr(obj, slot)
            val = fn(obj, *args)
            setattr(obj, slot, val)
            return val
    else:
        def memoized_fn(*args):
            if args not in memoized_fn.cache:
                memoized_fn.cache[args] = fn(*args)
            return memoized_fn.cache[args]

        memoized_fn.cache = {}
    return memoized_fn


class PriorityQueue:
    def __init__(self, order=min, f=lambda x: x):
        update(self, A=[], order=order, f=f)

    def append(self, item):
        bisect.insort(self.A, (self.f(item), item))

    def __len__(self):
        return len(self.A)

    def extend(self, items):
        for item in items:
            self.append(item)

    def pop(self):
        if self.order == min:
            return self.A.pop(0)[1]
        return self.A.pop()[1]


class Problem:
    def __init__(self, initial, goal=None):
        self.initial = initial
        self.goal = goal

    def successor(self, state):
        raise NotImplementedError

    def goal_test(self, state):
        return state == self.goal

    def path_cost(self, c, state1, action, state2):
        return c + 1


class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        update(self, state=state, parent=parent, action=action, path_cost=path_cost, depth=0)
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node %s>" % (self.state,)

    def path(self):
        x, result = self, [self]
        while x.parent:
            result.append(x.parent)
            x = x.parent
        return result

    def expand(self, problem):
        return [
            Node(next_state, self, act, problem.path_cost(self.path_cost, self.state, act, next_state))
            for (act, next_state) in problem.successor(self.state)
        ]

    def __eq__(self, other):
        return self.f == other.f

    def __ne__(self, other):
        return not (self == other)

    def __lt__(self, other):
        return self.f < other.f

    def __gt__(self, other):
        return self.f > other.f

    def __le__(self, other):
        return (self < other) or (self == other)

    def __ge__(self, other):
        return (self > other) or (self == other)


def graph_search(problem, fringe):
    closed = {}
    expanded = 0

    fringe.append(Node(problem.initial))
    while fringe:
        node = fringe.pop()
        if problem.goal_test(node.state):
            return node, expanded
        if node.state not in closed:
            closed[node.state] = True
            fringe.extend(node.expand(problem))
            expanded += 1
    return None


def best_first_graph_search(problem, f):
    f = memoize(f, 'f')
    return graph_search(problem, PriorityQueue(min, f))


def astar_search(problem, h=None):
    h = h or problem.h

    def f(node):
        return max(getattr(node, 'f', -infinity), node.path_cost + h(node))

    return best_first_graph_search(problem, f)


class ElevatorsProblem(Problem):

    ON_FLOOR = 0
    IN_ELEVATOR = 1

    def __init__(self, initial):
        self.height = initial["height"]

        self.elevator_ids = tuple(sorted(initial["Elevators"]))
        self.elevator_index = dict((eid, idx) for idx, eid in enumerate(self.elevator_ids))
        self.elevator_capacities = []
        self.elevator_reachable = []
        self.elevator_reachable_sets = []
        self.elevator_pickup_sets = []
        initial_elevators = []
        for eid in self.elevator_ids:
            current_floor, reachable_floors, max_weight = initial["Elevators"][eid]
            reachable_floors = tuple(sorted(reachable_floors))
            initial_elevators.append(current_floor)
            self.elevator_capacities.append(max_weight)
            self.elevator_reachable.append(reachable_floors)
            reachable_set = frozenset(reachable_floors)
            self.elevator_reachable_sets.append(reachable_set)
            # A passenger can BOARD an elevator at any floor in its
            # reachable set, OR at its starting floor (even when that floor
            # is outside the reachable set, e.g. an elevator parked at 4
            # whose reachable set is {8}).
            self.elevator_pickup_sets.append(reachable_set | frozenset((current_floor,)))
        self.elevator_capacities = tuple(self.elevator_capacities)
        self.elevator_reachable = tuple(self.elevator_reachable)
        self.elevator_reachable_sets = tuple(self.elevator_reachable_sets)
        self.elevator_pickup_sets = tuple(self.elevator_pickup_sets)

        elevator_shared_floors = []
        elevator_neighbors = []
        for left_idx, left_reach in enumerate(self.elevator_reachable_sets):
            shared_floors_row = []
            for right_idx in range(len(self.elevator_reachable_sets)):
                # Floors where a passenger can EXIT elevator left_idx (must be
                # in left's reachable set) and BOARD elevator right_idx (must
                # be in right's pickup set = reachable ∪ starting floor).
                # Using right's pickup set lets transfers happen at an
                # elevator's starting floor even when it is outside its R.
                shared = frozenset(left_reach & self.elevator_pickup_sets[right_idx])
                shared_floors_row.append(shared)
            elevator_shared_floors.append(tuple(shared_floors_row))
            # For the goal-backward BFS below: right_idx is a predecessor of
            # left_idx (one ride earlier toward the goal) iff a passenger can
            # transfer FROM right_idx INTO left_idx, i.e. some floor lies in
            # R_right ∩ pickup_left.
            neighbors = []
            for right_idx, right_reach in enumerate(self.elevator_reachable_sets):
                if right_idx != left_idx and (right_reach & self.elevator_pickup_sets[left_idx]):
                    neighbors.append(right_idx)
            elevator_neighbors.append(tuple(neighbors))
        self.elevator_shared_floors = tuple(elevator_shared_floors)
        self.elevator_neighbors = tuple(elevator_neighbors)

        self.person_ids = tuple(sorted(initial["Persons"]))
        self.person_index = dict((pid, idx) for idx, pid in enumerate(self.person_ids))
        self.person_weights = []
        self.person_goals = []
        initial_persons = []
        for pid in self.person_ids:
            start_floor, weight, goal_floor = initial["Persons"][pid]
            self.person_weights.append(weight)
            self.person_goals.append(goal_floor)
            initial_persons.append((self.ON_FLOOR, start_floor))
        self.person_weights = tuple(self.person_weights)
        self.person_goals = tuple(self.person_goals)

        self.person_goal_elevators = []
        self.person_useful_elevators = []
        self.person_goal_distance = []
        for person_idx in range(len(self.person_ids)):
            goal_elevators, useful_elevators, goal_distance = self._build_person_graph_data(person_idx)
            self.person_goal_elevators.append(goal_elevators)
            self.person_useful_elevators.append(useful_elevators)
            self.person_goal_distance.append(goal_distance)
        self.person_goal_elevators = tuple(self.person_goal_elevators)
        self.person_useful_elevators = tuple(self.person_useful_elevators)
        self.person_goal_distance = tuple(self.person_goal_distance)

        initial_state = (tuple(initial_elevators), tuple(initial_persons))
        Problem.__init__(self, initial_state)

    def _build_person_graph_data(self, person_idx):
        weight = self.person_weights[person_idx]
        goal = self.person_goals[person_idx]

        eligible = frozenset(
            elevator_idx
            for elevator_idx, capacity in enumerate(self.elevator_capacities)
            if capacity >= weight
        )
        goal_elevators = frozenset(
            elevator_idx
            for elevator_idx in eligible
            if goal in self.elevator_reachable_sets[elevator_idx]
        )

        goal_distance = [None] * len(self.elevator_ids)
        useful = set(goal_elevators)
        queue = deque(goal_elevators)
        for elevator_idx in goal_elevators:
            goal_distance[elevator_idx] = 1

        while queue:
            current = queue.popleft()
            for neighbor in self.elevator_neighbors[current]:
                if neighbor not in eligible or goal_distance[neighbor] is not None:
                    continue
                goal_distance[neighbor] = goal_distance[current] + 1
                useful.add(neighbor)
                queue.append(neighbor)

        return goal_elevators, frozenset(useful), tuple(goal_distance)

    def successor(self, state):
        elevator_floors, person_locations = state
        loads = [0] * len(self.elevator_ids)
        for person_idx, location in enumerate(person_locations):
            if location[0] == self.IN_ELEVATOR:
                loads[location[1]] += self.person_weights[person_idx]

        successors = []
        move_targets = [set() for _ in self.elevator_ids]

        for person_idx, location in enumerate(person_locations):
            location_type, value = location
            if location_type == self.ON_FLOOR:
                if value == self.person_goals[person_idx]:
                    continue
                for elevator_idx in self.person_useful_elevators[person_idx]:
                    if value in self.elevator_pickup_sets[elevator_idx]:
                        move_targets[elevator_idx].add(value)
            else:
                elevator_idx = value
                goal_floor = self.person_goals[person_idx]
                if goal_floor in self.elevator_reachable_sets[elevator_idx]:
                    move_targets[elevator_idx].add(goal_floor)
                for next_elevator_idx in self.person_useful_elevators[person_idx]:
                    if next_elevator_idx == elevator_idx:
                        continue
                    move_targets[elevator_idx].update(
                        self.elevator_shared_floors[elevator_idx][next_elevator_idx]
                    )

        for elevator_idx, elevator_id in enumerate(self.elevator_ids):
            current_floor = elevator_floors[elevator_idx]
            for target_floor in move_targets[elevator_idx]:
                if target_floor == current_floor:
                    continue
                next_elevator_floors = list(elevator_floors)
                next_elevator_floors[elevator_idx] = target_floor
                successors.append(
                    (
                        "MOVE{%s,%s}" % (elevator_id, target_floor),
                        (tuple(next_elevator_floors), person_locations),
                    )
                )

        for person_idx, person_id in enumerate(self.person_ids):
            location_type, value = person_locations[person_idx]
            if location_type == self.ON_FLOOR:
                floor = value
                if floor == self.person_goals[person_idx]:
                    continue
                for elevator_idx, elevator_id in enumerate(self.elevator_ids):
                    if elevator_floors[elevator_idx] != floor:
                        continue
                    if elevator_idx not in self.person_useful_elevators[person_idx]:
                        continue
                    if loads[elevator_idx] + self.person_weights[person_idx] > self.elevator_capacities[elevator_idx]:
                        continue
                    next_person_locations = list(person_locations)
                    next_person_locations[person_idx] = (self.IN_ELEVATOR, elevator_idx)
                    successors.append(
                        (
                            "ENTER{%s,%s}" % (person_id, elevator_id),
                            (elevator_floors, tuple(next_person_locations)),
                        )
                    )
            else:
                elevator_idx = value
                elevator_id = self.elevator_ids[elevator_idx]
                current_floor = elevator_floors[elevator_idx]
                next_person_locations = list(person_locations)
                next_person_locations[person_idx] = (self.ON_FLOOR, current_floor)
                successors.append(
                    (
                        "EXIT{%s,%s}" % (person_id, elevator_id),
                        (elevator_floors, tuple(next_person_locations)),
                    )
                )

        return successors

    def goal_test(self, state):
        _, person_locations = state
        for person_idx, location in enumerate(person_locations):
            if location[0] != self.ON_FLOOR or location[1] != self.person_goals[person_idx]:
                return False
        return True

    def h_astar(self, node):
        elevator_floors, person_locations = node.state
        max_person_cost = 0
        total_person_actions = 0
        max_move_actions = 0
        must_visit = set()

        for person_idx, location in enumerate(person_locations):
            person_cost, person_actions, move_actions = self._person_lower_bound(
                person_idx, location, elevator_floors
            )
            if person_cost > max_person_cost:
                max_person_cost = person_cost
            total_person_actions += person_actions
            if move_actions > max_move_actions:
                max_move_actions = move_actions

            # Collect floors an elevator still has to reach: the pickup floor
            # for an on-floor person not yet at their goal, and the drop-off
            # floor for every person not yet delivered.
            location_type, value = location
            goal_floor = self.person_goals[person_idx]
            if location_type == self.ON_FLOOR:
                if value != goal_floor:
                    must_visit.add(value)
                    must_visit.add(goal_floor)
            elif elevator_floors[value] != goal_floor:
                must_visit.add(goal_floor)

        # Each distinct must-visit floor not currently occupied by an elevator
        # needs at least one MOVE ending there.  This is an independent lower
        # bound on the number of MOVE actions; combine it with the per-person
        # move bound by taking the larger of the two (moves are shareable, so
        # we never sum them).
        occupied = set(elevator_floors)
        unvisited = 0
        for floor in must_visit:
            if floor not in occupied:
                unvisited += 1
        if unvisited > max_move_actions:
            max_move_actions = unvisited

        return max(max_person_cost, total_person_actions + max_move_actions)

    def _person_lower_bound(self, person_idx, location, elevator_floors):
        goal_floor = self.person_goals[person_idx]
        location_type, value = location

        if location_type == self.ON_FLOOR:
            if value == goal_floor:
                return 0, 0, 0
            min_rides = self._min_rides_from_floor(person_idx, value)
            if min_rides is None:
                return 0, 0, 0
            elevator_at_floor = False
            for elevator_idx in self.person_useful_elevators[person_idx]:
                if value not in self.elevator_pickup_sets[elevator_idx]:
                    continue
                if elevator_floors[elevator_idx] == value:
                    elevator_at_floor = True
                    break
            return (
                (2 * min_rides) + 1 + (0 if elevator_at_floor else 1),
                2 * min_rides,
                min_rides,
            )

        elevator_idx = value
        current_floor = elevator_floors[elevator_idx]
        if current_floor == goal_floor:
            return 1, 1, 0

        rides = self.person_goal_distance[person_idx][elevator_idx]
        if rides is None:
            return 0, 0, 0
        return 2 * rides, (2 * rides) - 1, max(1, rides - 1)

    def _min_rides_from_floor(self, person_idx, floor):
        min_rides = None
        for elevator_idx in self.person_useful_elevators[person_idx]:
            if floor not in self.elevator_pickup_sets[elevator_idx]:
                continue
            rides = self.person_goal_distance[person_idx][elevator_idx]
            if rides is None:
                continue
            if min_rides is None or rides < min_rides:
                min_rides = rides
        return min_rides

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
    def _planning_elevators(self, state, person_ids):
        if len(self._elevators) <= 1:
            return self._elevators

        elevators_t, persons_t, _ = state
        elev_floor = {eid: f for eid, f, _ in elevators_t}
        persons_by_id = {pid: loc for pid, loc in persons_t}

        ranked = sorted(self._elevators, key=lambda eid: self._elev_prob.get(eid, 1.0), reverse=True)
        anchor = ranked[0] if ranked else None
        if anchor is None or self._elev_prob.get(anchor, 1.0) < 0.9:
            return self._elevators

        anchor_reach = self.reachable[anchor]
        anchor_cap = self.capacities[anchor]
        for pid in person_ids:
            loc = persons_by_id[pid]
            goal = self._person_goal[pid]
            if goal not in anchor_reach or self._person_weight[pid] > anchor_cap:
                return self._elevators
            if loc[0] == "floor":
                if loc[1] not in anchor_reach:
                    return self._elevators
            elif loc[1] != anchor:
                return self._elevators
        return (anchor,)

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
        plan_elevators = self._planning_elevators(state, person_ids)

        elevators_def = {
            eid: (elev_floor[eid], tuple(sorted(self.reachable[eid])), self.capacities[eid])
            for eid in plan_elevators
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
            solved = astar_search(problem, h=problem.h_astar)
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
        if self._person_reward_mean.get(self._farming_pid, 0.0) < 40.0:
            return False
        steps_left = self.game.get_max_steps() - self.game.get_current_steps()
        if self._is_rl_like:
            return steps_left >= 2
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
                    score = (self._person_reward_mean[pid] * self._elev_prob[eid]) / (1 + abs(goal - ef))
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
                        score = (self._person_reward_mean[pid] * self._elev_prob[eid]) / (1 + abs(target - ef))
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
