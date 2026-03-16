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


pub trait ToF32 {
    type Output;
    fn to_f32(self) -> Self::Output;
}

impl ToF32 for crate::math::Vec3 {
    type Output = glam::Vec3;
    fn to_f32(self) -> glam::Vec3 {
        glam::Vec3::new(self.x as f32, self.y as f32, self.z as f32)
    }
}

impl ToF32 for crate::math::Quat {
    type Output = glam::Quat;
    fn to_f32(self) -> glam::Quat {
        glam::Quat::from_xyzw(
            self.x as f32, self.y as f32,
            self.z as f32, self.w as f32
        )
    }
}

impl ToF32 for crate::math::Mat4 {
    type Output = glam::Mat4;
    fn to_f32(self) -> glam::Mat4 {
        // column by column cast
        glam::Mat4::from_cols_array(&self.to_cols_array().map(|x| x as f32))
    }
}




pub trait ToFloat {
    type Output;
    fn to_float(self) -> Self::Output;
}

impl ToFloat for glam::Vec3 {
    type Output = crate::math::Vec3;
    fn to_float(self) -> crate::math::Vec3 {
        crate::math::Vec3::new(self.x as Float, self.y as Float, self.z as Float)
    }
}

impl ToFloat for glam::Quat {
    type Output = crate::math::Quat;
    fn to_float(self) -> crate::math::Quat {
        crate::math::Quat::from_xyzw(
            self.x as Float, self.y as Float,
            self.z as Float, self.w as Float
        )
    }
}

impl ToFloat for glam::Mat4 {
    type Output = crate::math::Mat4;
    fn to_float(self) -> crate::math::Mat4 {
        // column by column cast
        crate::math::Mat4::from_cols_array(&self.to_cols_array().map(|x| x as Float))
    }
}


pub trait ToGlamVec3 {
    type Output;
    fn to_glam_vec3(self) -> Self::Output;
}
impl ToGlamVec3 for crate::math::Vec3 {
    type Output = glam::Vec3;
    fn to_glam_vec3(self) -> glam::Vec3 {
        glam::Vec3::new(self.x as f32, self.y as f32, self.z as f32)
    }
}

pub trait ToGlamQuat {
    type Output;
    fn to_glam_quat(self) -> Self::Output;
}

impl ToGlamQuat for crate::math::Quat {
    type Output = glam::Quat;
    fn to_glam_quat(self) -> glam::Quat {
        glam::Quat::from_xyzw(self.x as f32, self.y as f32, self.z as f32, self.w as f32)
    }
}