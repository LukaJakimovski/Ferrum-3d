use crate::math::{Float, Mat3, Quat, Vec3};

#[inline] fn quat_add(a: Quat, b: Quat) -> Quat {
    Quat::from_xyzw(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w)
}
#[inline] fn quat_scale(q: Quat, s: Float) -> Quat {
    Quat::from_xyzw(q.x * s, q.y * s, q.z * s, q.w * s)
}

fn q_derivative(q: Quat, omega: Vec3) -> Quat {
    let omega_quat = Quat::from_xyzw(omega.x, omega.y, omega.z, 0.0);
    // dq/dt = 0.5 * omega_quat * q
   quat_scale(omega_quat * q, 0.5)
}
pub fn integrate_rk4(rotation: &mut Quat, omega: &mut Vec3, inertia: Mat3, inv_inertia: Mat3, torque_world: Vec3, dt: Float) {
    // Compute alpha once (treating omega as ~constant over the step)
    let r           = Mat3::from_quat(*rotation);
    let omega_body: Vec3  = r.transpose() * *omega;
    let torque_body = r.transpose() * torque_world;
    let gyro        = omega_body.cross(inertia * omega_body);
    let alpha_world = r * (inv_inertia * (torque_body - gyro));

    // Integrate omega (Euler is fine here; RK4 matters most for q)
    *omega += alpha_world * dt;

    // RK4 on quaternion
    let q  = *rotation;
    let w  = *omega;

    let k1 = q_derivative(q, w);
    let k2 = q_derivative(quat_add(q, quat_scale(k1, dt * 0.5)), w);
    let k3 = q_derivative(quat_add(q, quat_scale(k2, dt * 0.5)), w);
    let k4 = q_derivative(quat_add(q, quat_scale(k3, dt)), w);

    let delta = quat_scale(
        quat_add(quat_add(quat_add(k1, quat_scale(k2, 2.0)),
                          quat_scale(k3, 2.0)), k4),
        dt / 6.0,
    );

    *rotation = quat_add(q, delta).normalize();
}