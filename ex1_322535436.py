from collections import deque

import search as search
import utils as utils

id = ["322535436"]


class ElevatorsProblem(search.Problem):

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
        search.Problem.__init__(self, initial_state)

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
                    if value in self.elevator_reachable_sets[elevator_idx]:
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


def create_elevators_problem(game):
    print("<<create_elevators_problem")
    return ElevatorsProblem(game)


if __name__ == '__main__':
    import ex1_check
    ex1_check.main()
