"""Generate docs/Assignment2_Summary.pdf using ReportLab (pure Python).

Usage:
    pip install reportlab
    python docs/generate_summary_pdf.py
"""

import os
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_LEFT, TA_CENTER
from reportlab.platypus import (
    SimpleDocTemplate,
    Paragraph,
    Spacer,
    Table,
    TableStyle,
    ListFlowable,
    ListItem,
)

# Output path relative to this script's directory
OUTPUT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Assignment2_Summary.pdf")

PAGE_WIDTH, PAGE_HEIGHT = A4
MARGIN = 2.2 * cm


def build_styles():
    styles = getSampleStyleSheet()

    styles.add(ParagraphStyle(
        name="DocTitle",
        parent=styles["Title"],
        fontSize=18,
        leading=24,
        spaceAfter=12,
        alignment=TA_CENTER,
    ))
    styles.add(ParagraphStyle(
        name="H2",
        parent=styles["Heading2"],
        fontSize=13,
        leading=18,
        spaceBefore=14,
        spaceAfter=4,
    ))
    styles.add(ParagraphStyle(
        name="H3",
        parent=styles["Heading3"],
        fontSize=11,
        leading=15,
        spaceBefore=10,
        spaceAfter=3,
    ))
    styles.add(ParagraphStyle(
        name="Body",
        parent=styles["Normal"],
        fontSize=10,
        leading=14,
        spaceAfter=6,
    ))
    styles.add(ParagraphStyle(
        name="TableCell",
        parent=styles["Normal"],
        fontSize=9,
        leading=12,
        wordWrap="LTR",
    ))
    return styles


def table_style():
    return TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#4472C4")),
        ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
        ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
        ("FONTSIZE", (0, 0), (-1, 0), 9),
        ("ALIGN", (0, 0), (-1, 0), "CENTER"),
        ("BOTTOMPADDING", (0, 0), (-1, 0), 6),
        ("TOPPADDING", (0, 0), (-1, 0), 6),
        ("BACKGROUND", (0, 1), (-1, -1), colors.HexColor("#DCE6F1")),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.HexColor("#DCE6F1"), colors.white]),
        ("FONTNAME", (0, 1), (-1, -1), "Helvetica"),
        ("FONTSIZE", (0, 1), (-1, -1), 9),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("TOPPADDING", (0, 1), (-1, -1), 4),
        ("BOTTOMPADDING", (0, 1), (-1, -1), 4),
        ("LEFTPADDING", (0, 0), (-1, -1), 5),
        ("RIGHTPADDING", (0, 0), (-1, -1), 5),
        ("GRID", (0, 0), (-1, -1), 0.5, colors.HexColor("#4472C4")),
        ("BOX", (0, 0), (-1, -1), 1, colors.HexColor("#4472C4")),
    ])


def build_story(styles):
    usable_width = PAGE_WIDTH - 2 * MARGIN
    story = []

    # ------------------------------------------------------------------ #
    # Title
    # ------------------------------------------------------------------ #
    story.append(Paragraph(
        "Summary: Assignment 2 — Stochastic Multi-Elevator Controller",
        styles["DocTitle"],
    ))
    story.append(Paragraph(
        "A breakdown of what the assignment demanded and how <b>ex2.py</b> satisfies it.",
        styles["Body"],
    ))
    story.append(Spacer(1, 0.3 * cm))

    # ------------------------------------------------------------------ #
    # Section 1 — What the assignment asked for
    # ------------------------------------------------------------------ #
    story.append(Paragraph("1. What the assignment asked for", styles["H2"]))
    story.append(Paragraph(
        "Based on the stub, engine (<i>ext_elev.py</i>), grader (<i>ex2_check.py</i>), "
        "and README, the demands are:",
        styles["Body"],
    ))

    req_data = [
        [Paragraph("<b>Requirement</b>", styles["TableCell"]),
         Paragraph("<b>Source</b>", styles["TableCell"])],
        [Paragraph(
            "Implement <b>Controller.choose_next_action(state)</b> returning one legal action string per step",
            styles["TableCell"]),
         Paragraph("<i>ex2.py</i> stub", styles["TableCell"])],
        [Paragraph(
            "Return one of <b>MOVE{e,f}</b>, <b>ENTER{p,e}</b>, <b>EXIT{p,e}</b>, <b>RESET</b>",
            styles["TableCell"]),
         Paragraph("engine contract", styles["TableCell"])],
        [Paragraph(
            "State is <b>(elevators_t, persons_t, total_persons_remaining)</b>",
            styles["TableCell"]),
         Paragraph("engine", styles["TableCell"])],
        [Paragraph(
            "Interact <b>only</b> through <b>GameAPI</b> — touching the underlying Game = grade of zero",
            styles["TableCell"]),
         Paragraph("engine-access policy", styles["TableCell"])],
        [Paragraph(
            "Handle a <b>stochastic MDP</b>: elevator moves and person enter/exit can fail",
            styles["TableCell"]),
         Paragraph("<i>Game._apply_*</i>", styles["TableCell"])],
        [Paragraph(
            "Maximize reward across <b>33 problems</b> (11 layouts × easy/medium/hard) × 30 seeds, within a step horizon",
            styles["TableCell"]),
         Paragraph("<i>ex2_check.py</i>", styles["TableCell"])],
        [Paragraph(
            "Exploit that delivering the <b>last</b> person grants goal_reward and resets the layout (rewards can be re-farmed)",
            styles["TableCell"]),
         Paragraph("engine + README", styles["TableCell"])],
        [Paragraph(
            "Match/beat baselines: <b>random</b>, <b>sol1_h3/h5/h6</b>, <b>sol2</b>",
            styles["TableCell"]),
         Paragraph("<i>baseline/summary.md</i>", styles["TableCell"])],
    ]
    col_widths = [usable_width * 0.65, usable_width * 0.35]
    req_table = Table(req_data, colWidths=col_widths, repeatRows=1)
    req_table.setStyle(table_style())
    story.append(req_table)
    story.append(Spacer(1, 0.3 * cm))

    # ------------------------------------------------------------------ #
    # Section 2 — Overall approach
    # ------------------------------------------------------------------ #
    story.append(Paragraph("2. Overall approach", styles["H2"]))
    story.append(Paragraph(
        "The implementation is a <b>greedy, priority-ranked reactive controller</b> (not a planner/MDP solver). "
        "Each call picks the single best legal action by walking down a fixed priority ladder, with scores that "
        "fold in <b>expected reward</b>, <b>action success probabilities</b>, and <b>distance</b>. "
        "This is a pragmatic choice for a stochastic setting where full planning is expensive and outcomes are noisy.",
        styles["Body"],
    ))

    # ------------------------------------------------------------------ #
    # Section 3 — Initialization
    # ------------------------------------------------------------------ #
    story.append(Paragraph("3. Initialization &amp; precomputation (<code>__init__</code>)", styles["H2"]))
    story.append(Paragraph(
        "All static data is front-loaded via the API and cached so per-step decisions are cheap:",
        styles["Body"],
    ))
    story.append(ListFlowable([
        ListItem(Paragraph(
            "Pulls <b>get_reachable</b>, <b>get_capacities</b>, <b>get_goal_reward</b>, plus lazy caches "
            "for person goal/weight/reward-mean and the <b>stochastic probabilities</b> "
            "(<i>_person_prob</i>, <i>_elev_prob</i>).",
            styles["Body"]), leftIndent=15),
        ListItem(Paragraph(
            "Builds an <b>elevator transfer graph</b>: <i>_shared_floors</i> (floors two elevators have in "
            "common) and an adjacency list <i>_adj</i>, enabling multi-elevator handoffs when no single "
            "elevator reaches a goal.",
            styles["Body"]), leftIndent=15),
        ListItem(Paragraph(
            "Precomputes <b>reset-farming economics</b>: <i>_compute_loop_rps</i> finds the best "
            "reward-per-step single-person delivery loop, and a break-even threshold for when farming beats "
            "full delivery.",
            styles["Body"]), leftIndent=15),
    ], bulletType="bullet", start=None))

    # ------------------------------------------------------------------ #
    # Section 4 — Decision ladder
    # ------------------------------------------------------------------ #
    story.append(Paragraph("4. The decision ladder (<code>choose_next_action</code>)", styles["H2"]))
    story.append(Paragraph(
        "The action is chosen by the first rule that fires:",
        styles["Body"],
    ))

    ladder_data = [
        [Paragraph("<b>Step</b>", styles["TableCell"]),
         Paragraph("<b>Rule</b>", styles["TableCell"]),
         Paragraph("<b>Description</b>", styles["TableCell"])],
        [Paragraph("1", styles["TableCell"]),
         Paragraph("<b>EXIT at goal</b>", styles["TableCell"]),
         Paragraph("Drop a rider who is already on their goal floor (collect reward).", styles["TableCell"])],
        [Paragraph("2", styles["TableCell"]),
         Paragraph("<b>EXIT for transfer</b>", styles["TableCell"]),
         Paragraph(
             "If the current elevator can't reach a rider's goal, drop them on a shared floor toward a "
             "routed next elevator.",
             styles["TableCell"])],
        [Paragraph("3", styles["TableCell"]),
         Paragraph("<b>ENTER</b>", styles["TableCell"]),
         Paragraph(
             "Board a waiting person if capacity allows; scored by "
             "<i>reward_mean × person_prob × elev_prob</i>, with a large bonus when the elevator can reach "
             "the goal directly, and a filter to avoid boarding the wrong elevator on a transfer chain.",
             styles["TableCell"])],
        [Paragraph("4", styles["TableCell"]),
         Paragraph("<b>MOVE loaded</b>", styles["TableCell"]),
         Paragraph(
             "Drive a loaded elevator toward the nearest goal (or transfer floor), scored by reward and distance.",
             styles["TableCell"])],
        [Paragraph("5", styles["TableCell"]),
         Paragraph("<b>Reset-farm</b>", styles["TableCell"]),
         Paragraph(
             "<b>RESET</b> when looping a cheap high-reward delivery beats delivering low-value remainders.",
             styles["TableCell"])],
        [Paragraph("6", styles["TableCell"]),
         Paragraph("<b>MOVE empty</b>", styles["TableCell"]),
         Paragraph(
             "Send an empty elevator toward the most valuable reachable waiting person.",
             styles["TableCell"])],
        [Paragraph("7", styles["TableCell"]),
         Paragraph("<b>RESET</b>", styles["TableCell"]),
         Paragraph("Fallback.", styles["TableCell"])],
    ]
    col_widths_ladder = [usable_width * 0.07, usable_width * 0.22, usable_width * 0.71]
    ladder_table = Table(ladder_data, colWidths=col_widths_ladder, repeatRows=1)
    ladder_table.setStyle(table_style())
    story.append(ladder_table)
    story.append(Spacer(1, 0.3 * cm))

    # ------------------------------------------------------------------ #
    # Section 5 — Harder demands
    # ------------------------------------------------------------------ #
    story.append(Paragraph("5. How it meets the harder demands", styles["H2"]))
    story.append(ListFlowable([
        ListItem(Paragraph(
            "<b>Stochasticity:</b> success probabilities are multiplied into every ENTER/MOVE score, so "
            "unreliable (\"broken\") elevators on hard tiers are deprioritized when a reliable alternative "
            "exists; loop step-counts are inflated by <i>1/elev_prob</i>.",
            styles["Body"]), leftIndent=15),
        ListItem(Paragraph(
            "<b>Multi-elevator layouts (e.g., <i>_m3</i>):</b> the BFS router (<i>_route_next_elevator</i>) "
            "+ <i>_best_transfer_floor</i> enable passenger handoffs across elevators with disjoint "
            "reachable sets.",
            styles["Body"]), leftIndent=15),
        ListItem(Paragraph(
            "<b>Reset-farming layouts (<i>_rl</i>):</b> dynamic comparison of "
            "<i>farming_rps × steps_remaining</i> vs. current full-delivery value lets it switch into "
            "\"farm one lucrative person + RESET\" mode — exactly what the <i>rl</i> problems reward.",
            styles["Body"]), leftIndent=15),
    ], bulletType="bullet", start=None))

    # ------------------------------------------------------------------ #
    # Section 6 — Compliance notes
    # ------------------------------------------------------------------ #
    story.append(Paragraph("6. Compliance notes", styles["H2"]))
    story.append(ListFlowable([
        ListItem(Paragraph(
            "✅ Goes exclusively through <b>GameAPI</b> (<i>self.game.*</i>) — respects the engine-access policy.",
            styles["Body"]), leftIndent=15),
        ListItem(Paragraph(
            "✅ Emits only the four legal action formats.",
            styles["Body"]), leftIndent=15),
        ListItem(Paragraph(
            "✅ <b>id = [\"322535436\"]</b> is set.",
            styles["Body"]), leftIndent=15),
        ListItem(Paragraph(
            "The repo also carries the Assignment 1 solution "
            "(<i>ex1_322535436.py</i>, <i>search.py</i>, <i>utils.py</i>), which the controller doesn't "
            "import at runtime — the A2 controller is self-contained and greedy rather than reusing the "
            "A1 A* search.",
            styles["Body"]), leftIndent=15),
    ], bulletType="bullet", start=None))

    # ------------------------------------------------------------------ #
    # Section 7 — One thing worth checking
    # ------------------------------------------------------------------ #
    story.append(Paragraph("7. One thing worth checking", styles["H2"]))
    story.append(Paragraph(
        "The deterministic optimum from A1 is referenced by the grader as <i>optimal_a1</i> to scale "
        "horizons, but the A2 controller doesn't use the A1 planner. That's a valid design, but a hybrid "
        "(short-horizon planning for the transfer chains, greedy elsewhere) is the natural next step to push "
        "scores on the harder <i>sol2</i>-dominated layouts shown in <i>baseline/summary.md</i>.",
        styles["Body"],
    ))

    return story


def main():
    doc = SimpleDocTemplate(
        OUTPUT_PATH,
        pagesize=A4,
        leftMargin=MARGIN,
        rightMargin=MARGIN,
        topMargin=MARGIN,
        bottomMargin=MARGIN,
        title="Assignment 2 Summary — Stochastic Multi-Elevator Controller",
        author="Cartman625",
    )
    styles = build_styles()
    story = build_story(styles)
    doc.build(story)
    print(f"PDF written to: {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
