# DS685 — TurtleBot Kinematics

Assignment: https://aegean.ai/aiml-common/assignments/main/robotics-fall-2026/turtlebot-kinematics

Official notebook: https://github.com/pantelis/eng-ai-agents/blob/main/notebooks/assignments/topics/kinematics/turtlebot-kinematics/turtlebot-kinematics.ipynb

## Prepared work

- Written solutions for Questions 1–5 and draft interpretation sentences for Questions 6–8 are included in `prepare_notebook.py`.
- `kinematics.py` implements `diff_drive_qdot`, `world_to_body`, and `odometry`.
- All 11 local NumPy/pytest tests passed in the initial development environment.
- `prepare_notebook.py` downloads the original 37-cell instructor notebook and fills in only the student answer/TODO cells. It checks the original Git blob SHA before changes.

## Generate and test

From the repository root:

```bash
python assignments/turtlebot-kinematics/prepare_notebook.py
python -m pytest assignments/turtlebot-kinematics/tests -q
```

The generated file is `assignments/turtlebot-kinematics/turtlebot-kinematics.ipynb`. It is not yet executed.

## Important submission steps

1. Open the generated notebook in Colab and run *all cells* in order, including `newton[sim]==1.6.0` and `rerun-sdk==0.37.0` setup.
2. Confirm that each simulation-dependent provided check prints `passed`. Local unit tests do not validate the Newton simulator.
3. Inspect the simulation/path plots and adjust the draft observation sentences to match actual evidence.
4. Download `recordings/turtlebot.rrd` from Colab and place it in this assignment directory's `recordings/` folder.
5. Save the executed notebook including actual outputs, add the notebook and recording to Git, then push the branch.
6. Submit the GitHub *directory* URL, subject to Canvas instructions.

No simulation results, check outputs, or recording have been fabricated.
