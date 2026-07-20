import numpy as np

##Port of Rotation3.cpp / MATLAB.cpp's quat2euler/euler2quat -- plain numpy
##functions instead of the C++ MATLAB matrix class (numpy already gives us
##everything that class exists to provide). Only the quaternion (type=1)
##path of Rotation3::L321 is ported, since that's all modeling.py needs.


def quat2euler(q0123):
    """3-2-1 Euler angles [phi, theta, psi] (radians) from [q0,q1,q2,q3]."""
    q0, q1, q2, q3 = q0123
    phi = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
    val = 2 * (q0 * q2 - q3 * q1)
    if abs(val) > 1.0:
        theta = np.sign(val) * np.pi / 2.0
    else:
        theta = np.arcsin(val)
    psi = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
    return np.array([phi, theta, psi])


def euler2quat(ptp):
    """[q0,q1,q2,q3] from 3-2-1 Euler angles [phi, theta, psi] (radians)."""
    phi, theta, psi = ptp
    cphi, sphi = np.cos(phi / 2), np.sin(phi / 2)
    cthe, sthe = np.cos(theta / 2), np.sin(theta / 2)
    cpsi, spsi = np.cos(psi / 2), np.sin(psi / 2)
    q0 = cphi * cthe * cpsi + sphi * sthe * spsi
    q1 = sphi * cthe * cpsi - cphi * sthe * spsi
    q2 = cphi * sthe * cpsi + sphi * cthe * spsi
    q3 = cphi * cthe * spsi - sphi * sthe * cpsi
    return np.array([q0, q1, q2, q3])


def quat_to_T321(q0123):
    """Body->Inertial direction cosine matrix from [q0,q1,q2,q3]."""
    q0, q1, q2, q3 = q0123
    return np.array([
        [q0**2 + q1**2 - q2**2 - q3**2, 2 * (q1 * q2 - q0 * q3), 2 * (q0 * q2 + q1 * q3)],
        [2 * (q1 * q2 + q0 * q3), q0**2 - q1**2 + q2**2 - q3**2, 2 * (q2 * q3 - q0 * q1)],
        [2 * (q1 * q3 - q0 * q2), 2 * (q0 * q1 + q2 * q3), q0**2 - q1**2 - q2**2 + q3**2],
    ])


def rotate_body_to_inertial(vec_b, q0123):
    return quat_to_T321(q0123) @ vec_b


def rotate_inertial_to_body(vec_i, q0123):
    return quat_to_T321(q0123).T @ vec_i


def quat_kinematic_derivative(q0123, pqr):
    """dq0123/dt given current quaternion and body angular rate [p,q,r]."""
    q0, q1, q2, q3 = q0123
    p, q, r = pqr
    return np.array([
        (-p * q1 - q * q2 - r * q3) / 2.0,
        (p * q0 + r * q2 - q * q3) / 2.0,
        (q * q0 - r * q1 + p * q3) / 2.0,
        (r * q0 + q * q1 - p * q2) / 2.0,
    ])


if __name__ == '__main__':
    for phi, theta, psi in [(0, 0, 0), (0.1, -0.2, 0.3), (0.5, 0.4, -1.0)]:
        q = euler2quat([phi, theta, psi])
        back = quat2euler(q)
        assert np.allclose(back, [phi, theta, psi], atol=1e-9), (back, [phi, theta, psi])
    ##Body x-axis under a 90deg yaw should rotate to inertial y-axis
    q90 = euler2quat([0, 0, np.pi / 2])
    v = rotate_body_to_inertial(np.array([1.0, 0.0, 0.0]), q90)
    assert np.allclose(v, [0, 1, 0], atol=1e-9), v
    print('Rotation3.py OK')
