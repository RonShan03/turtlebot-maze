# TurtleBot Kinematics

This experiment compares differential-drive kinematics and wheel-angle odometry
with a Newton physics simulation of a TurtleBot3 Waffle.

[Assignment](https://aegean.ai/aiml-common/assignments/main/robotics-fall-2026/turtlebot-kinematics)
| [Notebook](turtlebot-kinematics.ipynb)
| [Rerun recording](recordings/turtlebot.rrd)

The notebook contains answers to Questions 1-8, saved outputs from all 16 code
cells, and three figures. All four provided checks print `passed`.

## Results

The experiments ran on 2026-10-28 using Python 3.11.9 on the CPU simulation
device, with `newton[sim]==1.6.0` and `rerun-sdk==0.37.0`.

| Experiment | Measurement |
| --- | --- |
| Q6: model vs simulation | Final position gap 0.008 m; heading gap 0.012 rad |
| Q7: turning rate | Model 0.230 rad/s; simulation 0.227 rad/s |
| Q7: forward speed | Model 0.132 m/s; simulation 0.131 m/s |
| Q7: largest sideways speed | 0.0003 m/s |
| Q8: short-course odometry | Final position error 0.004 m |

The 60 s winding drive produced position errors of 0.7, 2.7, 5.4, and 7.9 cm
at 8, 20, 40, and 60 s. The plot shows error growing overall with brief
decreases as successive errors partly cancel.

The recording contains the 8 s drive sampled at 60 fps and passes the Rerun
0.37.0 verifier. It can be replayed in the
[Rerun web viewer](https://app.rerun.io/version/0.37.0/).

## Reproduction

Install the dependencies in a Python 3.11 virtual environment, then run these
commands from this directory:

```bash
python -m pip install -r requirements.txt
python execute_notebook.py
python validate_submission.py
python -m rerun rrd verify recordings/turtlebot.rrd
python -m pytest tests -q
```

All 11 regression tests pass. The submission validator checks responses,
execution order, saved errors, grading outputs, figures, long-drive
measurements, and the recording's presence.

The provided simulation code and grading checks match the
[official starter notebook](https://github.com/pantelis/eng-ai-agents/blob/main/notebooks/assignments/topics/kinematics/turtlebot-kinematics/turtlebot-kinematics.ipynb),
Git blob `09f731258a51081e0707e19984aeffc217e7ea37`.
To repeat that comparison with a downloaded original:

```bash
python validate_submission.py --source /path/to/original.ipynb
```
