import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kinematics import (HALF_SEPARATION, WHEEL_RADIUS, diff_drive_qdot,
                        odometry, world_to_body)


def euler_integrate(q0, u_L, u_R, duration=1., dt=0.01):
    q = np.array(q0, dtype=float)
    for _ in range(int(round(duration / dt))):
        q += dt * diff_drive_qdot(q, u_L, u_R)
    return q


def test_q6_straight():
    np.testing.assert_allclose(euler_integrate([0, 0, 0], 5, 5), [0, 5 * WHEEL_RADIUS, 0], atol=1e-12)


def test_q6_spin_in_place():
    np.testing.assert_allclose(euler_integrate([0, 0, 0], -2, 2),
                               [2 * WHEEL_RADIUS / HALF_SEPARATION, 0, 0], atol=1e-12)


def test_q6_north():
    np.testing.assert_allclose(euler_integrate([np.pi / 2, 0, 0], 5, 5),
                               [np.pi / 2, 0, 5 * WHEEL_RADIUS], atol=1e-12)


def test_q6_left_turn_and_arc():
    v = WHEEL_RADIUS * 4
    omega = WHEEL_RADIUS * 2 / (2 * HALF_SEPARATION)
    np.testing.assert_allclose(diff_drive_qdot([0, 0, 0], 3, 5), [omega, v, 0])
    q = euler_integrate([0, 0, 0], 3, 5, duration=4, dt=1e-4)
    expected = [4 * omega, v / omega * np.sin(4 * omega), v / omega * (1 - np.cos(4 * omega))]
    np.testing.assert_allclose(q, expected, atol=1.5e-5)


def test_q7_world_to_body_basics():
    np.testing.assert_allclose(world_to_body(np.pi / 2, 0, 1), [1, 0], atol=1e-12)
    np.testing.assert_allclose(world_to_body(0, 0, 1), [0, 1], atol=1e-12)


def test_q7_vectorization_and_inverse():
    p = np.array([0, 0.4, np.pi / 2, np.pi, -1.2])
    forward = np.array([1, 0.3, 0.7, -1, 2.1])
    lateral = np.array([0.1, 0, -0.2, 1, 0])
    wx = forward * np.cos(p) - lateral * np.sin(p)
    wy = forward * np.sin(p) + lateral * np.cos(p)
    fx, ly = world_to_body(p, wx, wy)
    np.testing.assert_allclose(fx, forward, atol=1e-14)
    np.testing.assert_allclose(ly, lateral, atol=1e-14)


def test_q8_straight():
    angles = np.column_stack([np.linspace(0, 10, 11), np.linspace(0, 10, 11)])
    est = odometry([0, 0, 0], angles)
    assert est.shape == (11, 3)
    np.testing.assert_allclose(est[-1], [0, 10 * WHEEL_RADIUS, 0], atol=1e-12)


def test_q8_spin():
    angles = [[0, 0], [-1, 1], [-2, 2]]
    est = odometry([0, 0, 0], angles)
    np.testing.assert_allclose(est[-1], [2 * WHEEL_RADIUS / HALF_SEPARATION, 0, 0], atol=1e-12)


def test_q8_empty_or_single_reading():
    out = odometry([0.3, 0.1, -0.2], [[12, 8]])
    np.testing.assert_allclose(out, [[0.3, 0.1, -0.2]])


def test_q4_exponential_and_pose():
    v, omega, t = 0.5, -np.pi / 4, 2
    a = omega * t
    E = np.array([[np.cos(a), -np.sin(a), v / omega * np.sin(a)],
                  [np.sin(a), np.cos(a), v / omega * (1-np.cos(a))],
                  [0, 0, 1]])
    T0 = np.array([[-1, 0, 2], [0, -1, 1], [0, 0, 1]])
    np.testing.assert_allclose(T0 @ E, [[0, -1, 2-2/np.pi], [1, 0, 1+2/np.pi], [0, 0, 1]], atol=1e-12)
