"""Execute every assignment cell and save its real outputs in place.

Install the instructor's simulation dependencies plus nbclient, nbformat,
ipykernel and matplotlib in the Python environment running this script.
The temporary kernel uses that same interpreter and a headless plot backend.
Provided notebook cells and grading tolerances are never modified.
"""

import argparse
import json
import sys
import tempfile
from pathlib import Path

import nbformat
from jupyter_client import KernelManager
from jupyter_client.kernelspec import KernelSpecManager
from nbclient import NotebookClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("notebook", nargs="?", type=Path,
                        default=Path(__file__).with_name("turtlebot-kinematics.ipynb"))
    args = parser.parse_args()
    path = args.notebook.resolve()
    notebook = nbformat.read(path, as_version=4)

    with tempfile.TemporaryDirectory(prefix="turtlebot-kernel-") as temporary:
        kernel_dir = Path(temporary) / "turtlebot"
        kernel_dir.mkdir()
        (kernel_dir / "kernel.json").write_text(json.dumps({
            "argv": [sys.executable, "-m", "ipykernel_launcher", "-f", "{connection_file}"],
            "display_name": "TurtleBot Python", "language": "python",
            "env": {"MPLBACKEND": "module://matplotlib_inline.backend_inline"},
        }), encoding="utf-8")
        manager = KernelManager(kernel_name="turtlebot",
                                kernel_spec_manager=KernelSpecManager(kernel_dirs=[temporary]))

        def report_start(cell, cell_index, **kwargs):
            if cell.cell_type == "code":
                print(f"Running cell {cell_index + 1}/{len(notebook.cells)}", flush=True)

        def report_complete(cell, cell_index, **kwargs):
            nbformat.write(notebook, path)
            for output in cell.get("outputs", []):
                if output.output_type == "stream":
                    print(output.text, end="", flush=True)

        client = NotebookClient(notebook, km=manager, timeout=1800,
                                resources={"metadata": {"path": str(path.parent)}},
                                on_cell_start=report_start, on_cell_executed=report_complete)
        try:
            client.execute(cleanup_kc=True)
        finally:
            nbformat.write(notebook, path)
    print(f"Saved executed notebook: {path}")


if __name__ == "__main__":
    main()
