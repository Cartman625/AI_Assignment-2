import ext_elev

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
        # Cache static person attributes (goals and weights never change).
        self._goal_cache = {}
        self._weight_cache = {}

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

        # 2. ENTER a waiting person into an elevator on the same floor.
        for eid, ef in elev_floor.items():
            if ef not in waiting:
                continue
            ew = elev_weight[eid]
            cap = self.capacities[eid]
            for pid in waiting[ef]:
                if ew + self._weight(pid) <= cap:
                    return f"ENTER{{{pid},{eid}}}"

        # 3. MOVE a loaded elevator toward its passengers' nearest goal.
        for eid, pids in riding.items():
            ef = elev_floor[eid]
            pending_goals = [self._goal(pid) for pid in pids
                             if self._goal(pid) != ef]
            if not pending_goals:
                continue
            target = min(pending_goals, key=lambda g: abs(g - ef))
            nf = self._closest_reachable(eid, ef, target)
            if nf is not None:
                return f"MOVE{{{eid},{nf}}}"

        # 4. MOVE an empty elevator toward the nearest waiting person.
        if not waiting:
            return "RESET"

        for eid, ef in elev_floor.items():
            if eid in riding:
                continue
            target = min(waiting.keys(), key=lambda f: abs(f - ef))
            if target == ef:
                continue
            nf = self._closest_reachable(eid, ef, target)
            if nf is not None:
                return f"MOVE{{{eid},{nf}}}"

        return "RESET"
