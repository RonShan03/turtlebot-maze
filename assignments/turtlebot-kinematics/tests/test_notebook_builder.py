import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from prepare_notebook import RESPONSES, cell_source, transform_notebook


def test_notebook_patch_preserves_other_cells_and_checks():
    nb = {"cells": []}
    for i in range(37):
        s = "provided content untouched\n"
        if i in RESPONSES:
            s = "Question 1\n" if i == 12 else f"Question {i} or In one sentence\n"
            if i in (13, 14, 15, 16):
                s = f"Question {i - 11}\n"
            if i in (23, 27, 36):
                s = "In one sentence\n"
            s += "*Write your answer here.*\n"
        if i == 18:
            s = "def diff_drive_qdot(q, u_L, u_R):\n    raise NotImplementedError\n"
        if i == 25:
            s = "def world_to_body(phi, x_dot, y_dot):\n    raise NotImplementedError\n"
        if i == 30:
            s = "def odometry(q0, wheel_angles):\n    raise NotImplementedError\n"
        nb["cells"].append({"cell_type": "code" if i in (18, 25, 30) else "markdown", "source": s.splitlines(True),
                            "execution_count": None, "outputs": []} if i in (18, 25, 30) else {"cell_type": "markdown", "source": s.splitlines(True)})
    unchanged_sources = {i: cell_source(c) for i,c in enumerate(nb["cells"]) if i not in {*RESPONSES, 18, 25, 30}}
    out = transform_notebook(nb)
    for i, src in unchanged_sources.items():
        assert cell_source(out["cells"][i]) == src
    for i in RESPONSES:
        assert "**Answer.**" in cell_source(out["cells"][i])
        assert "*Write your answer here.*" not in cell_source(out["cells"][i])
    for i in (18, 25, 30):
        assert "raise NotImplementedError" not in cell_source(out["cells"][i])
