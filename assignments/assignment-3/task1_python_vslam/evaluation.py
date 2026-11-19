"""Global Umeyama Sim(3)/SE(3), ATE translation and fixed-frame-interval RPE."""
import numpy as np
from .geometry import inverse


def align_poses(estimate, truth, with_scale=True):
    if len(estimate) != len(truth) or len(estimate) < 3:
        raise ValueError("alignment requires at least three corresponding poses")
    x, y = estimate[:, :3, 3], truth[:, :3, 3]
    mx, my = x.mean(axis=0), y.mean(axis=0)
    xc, yc = x - mx, y - my
    variance = np.mean(np.sum(xc ** 2, axis=1))
    if variance < 1e-12:
        raise ValueError("stationary estimate cannot be aligned")
    u, singular, vt = np.linalg.svd(yc.T @ xc / len(x))
    sign = np.ones(3)
    sign[-1] = np.sign(np.linalg.det(u @ vt))
    R = u @ np.diag(sign) @ vt
    scale = float(np.dot(singular, sign) / variance) if with_scale else 1.0
    t = my - scale * R @ mx
    aligned = np.array(estimate, copy=True)
    aligned[:, :3, :3] = R @ estimate[:, :3, :3]
    aligned[:, :3, 3] = scale * x @ R.T + t
    return aligned, {"scale": scale, "rotation": R.tolist(), "translation": t.tolist(),
                     "position_rank": int(np.linalg.matrix_rank(xc)),
                     "fit": "global least-squares Sim(3)" if with_scale else "global least-squares SE(3)"}


def metrics(estimate, truth, delta=1, valid=None):
    if delta < 1:
        raise ValueError("RPE delta must be positive")
    valid = np.ones(len(estimate), bool) if valid is None else np.asarray(valid, bool)
    residual = np.linalg.norm(estimate[:, :3, 3] - truth[:, :3, 3], axis=1)
    trans, angles = [], []
    for i in range(len(estimate) - delta):
        j = i + delta
        if not valid[i:j + 1].all():
            continue
        err = inverse(inverse(truth[i]) @ truth[j]) @ (inverse(estimate[i]) @ estimate[j])
        trans.append(np.linalg.norm(err[:3, 3]))
        angles.append(np.degrees(np.arccos(np.clip((np.trace(err[:3, :3]) - 1) / 2, -1, 1))))
    rmse = lambda values: float(np.sqrt(np.mean(np.square(values)))) if len(values) else None
    return {"ate_rmse_m": rmse(residual[valid]), "ate_all_including_held_rmse_m": rmse(residual),
            "ate_median_m": float(np.median(residual[valid])) if valid.any() else None,
            "rpe_translation_rmse_m": rmse(trans), "rpe_rotation_rmse_deg": rmse(angles),
            "rpe_delta_frames": delta, "rpe_pairs": len(trans)}
