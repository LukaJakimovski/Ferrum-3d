#[cfg(feature = "f64")]
pub use glam::{DVec2 as Vec2, DVec3 as Vec3, DVec4 as Vec4,
               DMat2 as Mat2, DMat3 as Mat3, DMat4 as Mat4,
               DQuat as Quat};
#[cfg(feature = "f64")]
    pub type Float = f64;

#[cfg(not(feature = "f64"))]
pub use glam::{Vec2, Vec3, Vec4, Mat2, Mat3, Mat4, Quat};
#[cfg(not(feature = "f64"))]
pub type Float = f32;