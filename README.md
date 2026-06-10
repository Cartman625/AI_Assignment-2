# AI Course — Assignment 2 (Stochastic Multi-Elevator Controller)

This repository contains the working set for **Assignment 2**: implementing a
controller for the stochastic multi-elevator MDP. It builds on the Assignment 1
deterministic search solution.

## Files
- `ext_elev.py` — the engine + `GameAPI` (the ONLY object the controller may touch).
- `ex2.py` — the controller stub to implement (`Controller.choose_next_action`).
- `ex2_check.py` — the grader. Runs 33 problems (11 layouts × easy/medium/hard) × 30 seeds.
- `ex2_random.py` — random baseline controller (shows how to enumerate legal actions).
- `ex1_322535436.py` — the student's Assignment 1 solution (`ElevatorsProblem` + `h_astar`).
- `search.py`, `utils.py` — the AIMA-style search framework used by the Assignment 1 solution.
- `baseline/summary.md`, `baseline/summary_tables.md` — benchmark scores for `random`, `sol1_h3/h5/h6`, `sol2` (targets to match/beat).

## Engine behavior (ground truth = `ext_elev.py`)
State passed to `choose_next_action` is `(elevators_t, persons_t, total_persons_remaining)`.
Actions: `MOVE{e,f}`, `ENTER{p,e}`, `EXIT{p,e}`, `RESET`. Each action consumes one
step; the episode ends after `get_max_steps()` steps. Delivering a person on their
goal floor yields a sampled reward; delivering the **last** person adds `goal_reward`
and resets the layout (rewards can be farmed again within the horizon).

> **Engine-access policy:** the controller must interact ONLY through `GameAPI`.
> Reaching into the underlying `Game` object is forbidden.

## How to run
```bash
python ex2_check.py
```

## Documentation

A formatted PDF summary of the Assignment 2 implementation is available at
[`docs/Assignment2_Summary.pdf`](docs/Assignment2_Summary.pdf).

It covers the assignment requirements, overall approach, initialization logic,
the decision ladder, and compliance notes.

To regenerate the PDF:
```bash
pip install reportlab
python docs/generate_summary_pdf.py
```
