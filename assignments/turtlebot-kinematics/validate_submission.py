"""Check the saved notebook and recording against the assignment deliverables.

Pass --source with the official starter notebook to additionally verify its
Git blob identity and that every provided cell and check remains unchanged.
This checks saved execution evidence; it does not rerun the physics.
"""

import argparse
import json
from pathlib import Path

from prepare_notebook import RESPONSES, UPSTREAM_BLOB_SHA, cell_source, git_blob_sha

BASE = Path(__file__).resolve().parent
STUDENT_CELLS = {*RESPONSES, 18, 25, 30}


def validate(directory, source=None):
    notebook = json.loads((directory / "turtlebot-kinematics.ipynb").read_text(encoding="utf-8"))
    cells = notebook["cells"]
    assert len(cells) == 37, "Expected the complete 37-cell instructor notebook"
    if source is not None:
        raw = source.read_bytes()
        assert git_blob_sha(raw) == UPSTREAM_BLOB_SHA, "Unrecognized official starter notebook"
        original = json.loads(raw)
        for i, cell in enumerate(cells):
            if i not in STUDENT_CELLS:
                assert cell_source(cell) == cell_source(original["cells"][i]), f"Provided cell {i} changed"

    for i in RESPONSES:
        text = cell_source(cells[i])
        assert "**Answer.**" in text and "*Write your answer here.*" not in text, f"Missing answer in cell {i}"

    code_cells = [cell for cell in cells if cell["cell_type"] == "code"]
    assert [cell["execution_count"] for cell in code_cells] == list(range(1, len(code_cells) + 1)), \
        "Run every code cell in order and save the execution counts"
    for cell in code_cells:
        assert all(out["output_type"] != "error" for out in cell.get("outputs", [])), "Saved execution error"

    for i in (19, 21, 26, 32):
        text = "".join("".join(out.get("text", [])) for out in cells[i].get("outputs", [])
                       if out["output_type"] == "stream")
        assert "passed" in text.splitlines(), f"No passing grading output in cell {i}"
    for i in (9, 22, 35):
        assert any("image/png" in out.get("data", {}) for out in cells[i].get("outputs", [])), \
            f"Missing saved plot in cell {i}"
    text = "".join("".join(out.get("text", [])) for out in cells[34].get("outputs", []))
    for seconds in (8, 20, 40, 60):
        assert f"odometry error after {seconds:2d} s:" in text, "Missing long-drive measurements"

    recording = directory / "recordings" / "turtlebot.rrd"
    assert recording.is_file() and recording.stat().st_size > 0, "Missing Rerun recording"
    print(f"Submission checks passed: 8 answers, {len(code_cells)} executed code cells, "
          "4 passing checks, 3 plots, 60 s experiment, recording")
    if source is not None:
        print("All instructor-provided cells and grading checks match the verified starter")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, default=BASE)
    parser.add_argument("--source", type=Path)
    args = parser.parse_args()
    validate(args.directory, args.source)


if __name__ == "__main__":
    main()
