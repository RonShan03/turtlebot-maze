"""Build the original DS685 TurtleBot assignment notebook with answers inserted.

The original notebook is downloaded from the course's public GitHub source;
all Provided simulation code and grading checks are preserved unchanged.

Usage (from repository root):
    python assignments/turtlebot-kinematics/prepare_notebook.py

If the course notebook is already downloaded:
    python assignments/turtlebot-kinematics/prepare_notebook.py --source /path/to/original.ipynb

This prepares source code; only running the notebook in Colab/Newton will
produce executed check outputs and recordings/turtlebot.rrd.
"""

import argparse
import ast
import hashlib
import json
from pathlib import Path
from urllib.request import urlopen

BASE = Path(__file__).resolve().parent
UPSTREAM_URL = (
    "https://raw.githubusercontent.com/pantelis/eng-ai-agents/main/"
    "notebooks/assignments/topics/kinematics/turtlebot-kinematics/turtlebot-kinematics.ipynb"
)
# Original notebook identified on 2026-10-28. Fail closed on changed official source.
UPSTREAM_BLOB_SHA = "09f731258a51081e0707e19984aeffc217e7ea37"

RESPONSES = {
    12: r"""**Answer.** The chassis has three degrees of freedom on a plane: two translations $(x,y)$ and one rotation $\phi$. Its configuration space is $\mathbb{R}^2\times S^1$ (equivalently, $SE(2)$). Heading belongs to $S^1$, because angles that differ by a full $2\pi$ turn describe the same orientation rather than different configurations.""",
    13: r"""**Answer.** Because $\hat y_b=(-\sin\phi,\cos\phi)$, the no-sideways-slip constraint is $-\sin\phi\,\dot x+\cos\phi\,\dot y=0$, or $A(q)\dot q=0$ with $A(q)=\begin{bmatrix}0&-\sin\phi&\cos\phi\end{bmatrix}$ for $q=(\phi,x,y)$. This nonholonomic constraint forbids instantaneous motion sideways but not reaching a sideways-displaced pose through sequences of forward/backward driving and turning.""",
    14: r"""**Answer.** Solving $ru_L=v-\omega d$ and $ru_R=v+\omega d$ yields $u_L=(v-\omega d)/r$ and $u_R=(v+\omega d)/r$. The robot turns in place when $u_L=-u_R\neq0$ (so $v=0$), and its signed turning radius is $\rho=v/\omega=d\dfrac{u_L+u_R}{u_R-u_L}$ when $u_R\ne u_L$; equal wheel speeds give straight motion with an infinite radius.""",
    15: r"""**Answer.** With $t=2\ \mathrm{s}$, $\Delta\phi=\omega t=-\pi/2$ and $v/\omega=-2/\pi\ \mathrm{m}$. Therefore,

$$
\exp(\hat\xi t)=\begin{bmatrix}
0&1&2/\pi\\
-1&0&-2/\pi\\
0&0&1
\end{bmatrix},\qquad
T_0=\begin{bmatrix}-1&0&2\\0&-1&1\\0&0&1\end{bmatrix},
$$

$$
T(2)=T_0\exp(\hat\xi t)=\begin{bmatrix}
0&-1&2-2/\pi\\
1&0&1+2/\pi\\
0&0&1
\end{bmatrix}.
$$

The final configuration is $(\phi,x,y)=(\pi/2,\ 2-2/\pi,\ 1+2/\pi)\approx(1.571\ \mathrm{rad},\ 1.363\ \mathrm{m},\ 1.637\ \mathrm{m})$. Since $\omega<0$ the robot turns right, following a clockwise circular arc; the product is $T_0\exp(\hat\xi t)$ because the twist is expressed in body coordinates and its displacement must be transformed by the initial world pose, whereas pre-multiplication would apply a spatial/world-frame displacement.""",
    16: r"""**Answer.** Odometry integrates successive wheel-angle increments, so errors from encoder noise, wheel-radius and separation uncertainty, and wheel slip accumulate in later position and heading estimates; an early heading error also misdirects all subsequent translations. Robots therefore fuse wheel odometry with independent observations from devices such as IMUs, cameras, LiDAR, GPS (when available), or map-based localization/SLAM to correct drift.""",
    23: r"""**Answer.** The simulated chassis should follow the no-slip differential-drive path closely, while finite motor response, contact/friction effects, or wheel slip can cause its trajectory to drift from the ideal kinematic prediction.""",
    27: r"""**Answer.** The near-zero body-frame lateral speed confirms the nonholonomic no-sideways-motion constraint $v_y=0$, because the wheels roll forward along their planes rather than sliding sideways across the floor.""",
    36: r"""**Answer.** Over a long winding drive the wheel-based odometry can drift from the simulated true path because small slip and imperfect wheel-motion estimates are integrated across many steps, although the instantaneous position error need not increase monotonically.""",
}


def git_blob_sha(raw):
    return hashlib.sha1(b"blob " + str(len(raw)).encode() + b"\0" + raw).hexdigest()


def function_source(function_name):
    source = (BASE / "kinematics.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    node = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == function_name)
    return ast.get_source_segment(source, node) + "\n"


def cell_source(cell):
    return "".join(cell["source"]) if isinstance(cell["source"], list) else cell["source"]


def set_cell_source(cell, source):
    cell["source"] = source.splitlines(keepends=True)


def transform_notebook(notebook):
    cells = notebook.get("cells", [])
    if len(cells) != 37:
        raise ValueError(f"Expected exactly 37 cells from verified original notebook, found {len(cells)}")
    expected_markers = {
        12: "Question 1", 13: "Question 2", 14: "Question 3",
        15: "Question 4", 16: "Question 5", 18: "def diff_drive_qdot",
        23: "In one sentence", 25: "def world_to_body", 27: "In one sentence",
        30: "def odometry", 36: "In one sentence",
    }
    for idx, marker in expected_markers.items():
        if marker not in cell_source(cells[idx]):
            raise ValueError(f"Original cell {idx} did not contain expected marker {marker!r}; refusing to patch")
    for idx in RESPONSES:
        src = cell_source(cells[idx])
        if "*Write your answer here.*" not in src:
            raise ValueError(f"Cell {idx} no longer has the original answer placeholder")
        set_cell_source(cells[idx], src.replace("*Write your answer here.*", RESPONSES[idx]))
    for idx, fn in ((18, "diff_drive_qdot"), (25, "world_to_body"), (30, "odometry")):
        # Replacing only the student TODO cells; Provided cells and check cells remain intact.
        set_cell_source(cells[idx], function_source(fn))
        cells[idx]["execution_count"] = None
        cells[idx]["outputs"] = []
    return notebook


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, help="Path to official original notebook (optional)")
    parser.add_argument("--output", type=Path, default=BASE / "turtlebot-kinematics.ipynb")
    parser.add_argument("--accept-upstream-change", action="store_true", help="Allow different official source SHA (still checks cell markers)")
    args = parser.parse_args()
    if args.source is None:
        print("Fetching official starter notebook:", UPSTREAM_URL)
        with urlopen(UPSTREAM_URL, timeout=30) as res:
            raw = res.read()
    else:
        raw = args.source.read_bytes()
    original_sha = git_blob_sha(raw)
    if not args.accept_upstream_change and original_sha != UPSTREAM_BLOB_SHA:
        raise SystemExit(f"Course notebook changed (git blob SHA {original_sha}); expected {UPSTREAM_BLOB_SHA}. "
                         "Review changes before using --accept-upstream-change.")
    nb = transform_notebook(json.loads(raw.decode("utf-8")))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(nb, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    print("Wrote:", args.output)
    print("Inserted written answers Q1-Q5, observations Q6-Q8, and graded code Q6-Q8.")
    print("WARNING: Newton/Warp simulations and Rerun recordings have not been executed.")


if __name__ == "__main__":
    main()
