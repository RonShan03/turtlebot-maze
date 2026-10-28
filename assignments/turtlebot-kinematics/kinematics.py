"""Solutions to the TurtleBot kinematics notebook's Questions 6-8.

Only the three graded functions in this module are copied into the notebook.
The other code supports local mathematical testing without Newton/Warp.
Pose convention: q = (heading in rad, world x in m, world y in m).
"""

import numpy as np

WHEEL_RADIUS = 0.033
HALF_SEPARATION = 0.1435


def diff_drive_qdot(q, u_L, u_R):
    """Return (phi_dot, x_dot, y_dot) in world coordinates."""
    phi = q[0]
    v = WHEEL_RADIUS * (u_L + u_R) / 2.0
    omega = WHEEL_RADIUS * (u_R - u_L) / (2.0 * HALF_SEPARATION)
    return np.array([omega, v * np.cos(phi), v * np.sin(phi)])


def world_to_body(phi, x_dot, y_dot):
    """Rotate world-frame planar velocity into forward/left components.

    Works with scalar or broadcastable array inputs.
    """
    c, s = np.cos(phi), np.sin(phi)
    v_x = c * x_dot + s * y_dot
    v_y = -s * x_dot + c * y_dot
    return v_x, v_y


def displace(q, twist):
    """Provided helper reproduced verbatim for local testing (not injected)."""
    w, vx, vy = twist
    if abs(w) < 1e-12:
        dx_b, dy_b = vx, vy
    else:
        dx_b = (vx * np.sin(w) + vy * (np.cos(w) - 1)) / w
        dy_b = (vy * np.sin(w) + vx * (1 - np.cos(w))) / w
    c, s = np.cos(q[0]), np.sin(q[0])
    return np.asarray(q, float) + np.array([w, c * dx_b - s * dy_b, s * dx_b + c * dy_b])


def odometry(q0, wheel_angles):
    """Integrate wheel-angle increments into all (phi, x, y) poses."""
    F = WHEEL_RADIUS * np.array([[-1 / (2 * HALF_SEPARATION), 1 / (2 * HALF_SEPARATION)],
                                 [0.5, 0.5], [0.0, 0.0]])
    path = [np.asarray(q0, float)]
    for delta_theta in np.diff(np.asarray(wheel_angles, float), axis=0):
        twist = F @ delta_theta
        path.append(displace(path[-1], twist))
    return np.array(path)
