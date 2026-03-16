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
    let deriv = |q: Quat, w: Vec3| -> (Quat, Vec3) {
        let r = Mat3::from_quat(q.normalize());
        let omega_body = r.transpose() * w;
        let torque_body = r.transpose() * torque_world;
        let gyro = omega_body.cross(inertia * omega_body);
        let alpha_body = inv_inertia * (torque_body - gyro);
        let alpha_world = r * alpha_body;
        let dq = q_derivative(q, w);
        (dq, alpha_world)
    };

    let (dq1, dw1) = deriv(*rotation, *omega);

    let (dq2, dw2) = deriv(
        quat_add(*rotation, quat_scale(dq1, dt * 0.5)),
        *omega + dw1 * dt * 0.5,
    );

    let (dq3, dw3) = deriv(
        quat_add(*rotation, quat_scale(dq2, dt * 0.5)),
        *omega + dw2 * dt * 0.5,
    );

    let (dq4, dw4) = deriv(
        quat_add(*rotation, quat_scale(dq3, dt)),
        *omega + dw3 * dt,
    );

    *omega += (dw1 + 2.0*dw2 + 2.0*dw3 + dw4) * (dt / 6.0);

    let delta_q = quat_scale(
        quat_add(quat_add(quat_add(dq1, quat_scale(dq2, 2.0)),
                          quat_scale(dq3, 2.0)), dq4),
        dt / 6.0,
    );
    *rotation = quat_add(*rotation, delta_q).normalize();
}