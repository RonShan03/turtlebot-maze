# Codex handoff — DS685 TurtleBot Kinematics

Repository: `RonShan03/turtlebot-maze`
Branch: `ds685/turtlebot-kinematics`
Base: `main` at `73bd2a32d7790ccb7de9fac0355ff99504a6df54`

## Completed

- Written answers to Q1–5 and draft observation sentences for Q6–8 in `prepare_notebook.py`.
- Differential-drive configuration rates, body-frame velocity, and odometry in `kinematics.py`.
- Local mathematical regression tests: 11 passed in development.
- Notebook preparation preserves provided physics simulation/check cells and validates the original 37-cell source and Git blob SHA.

## To finish

1. Switch to this branch and run `python assignments/turtlebot-kinematics/prepare_notebook.py`.
2. Run `python -m pytest assignments/turtlebot-kinematics/tests -q`.
3. Open the generated notebook in Colab; run all cells, including Newton/Warp physics simulations.
4. Verify Q6, Q7, Q8 provided checks print `passed`, preserve real output cells, and examine plots.
5. Review the draft explanations against measured results, especially whether the 60s odometry position error is monotonic (do not assert monotonicity without evidence).
6. Download actual Rerun recording `recordings/turtlebot.rrd`; commit the executed notebook and recorded file, and push.
7. Submit the full assignment directory URL, not the repo root.

Important: do not change the instructor's Provided grading checks or fabricate simulated values, Rerun recordings, or execution outputs. Neither the Newton physics checks nor the Rerun recording have been run/generated in this environment.
