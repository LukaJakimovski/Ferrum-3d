use ferrum_core::math::{Float, Quat, Vec3};

#[derive(Default)]
#[derive(Clone)]
pub struct RigidBody {
    pub(crate) position:     Vec3,    // hot - read every frame
    pub(crate) velocity:    Vec3,    // hot - read every frame
    pub(crate) orientation:  Quat,    // hot - read every frame
    pub(crate) _force:        Vec3,    // hot - written every frame
    pub(crate) _inv_mass:      Float,     // warm
    pub(crate) _inertia:       Vec3,    // warm
    pub(crate) _restitution:   Float,     // cold - only on collision
    pub(crate) _is_sleeping:   bool,    // cold
}

impl RigidBody {

    pub fn builder() -> Self {
        let mut body: RigidBody = Default::default();
        body.position = Vec3::ZERO;
        body.velocity = Vec3::ZERO;
        body.orientation = Quat::IDENTITY;
        body._force = Vec3::ZERO;
        body._inv_mass = 0.0;
        body._inertia = Vec3::ZERO;
        body._restitution = 0.0;
        body._is_sleeping = false;
        body
    }
    #[allow(unused)]
    pub fn position(mut self, position: Vec3) -> Self {
        self.position = position;
        self
    }
    #[allow(unused)]
    pub fn velocity(mut self, velocity: Vec3) -> Self {
        self.velocity = velocity;
        self
    }
    #[allow(unused)]
    pub fn orientation(mut self, orientation: Quat) -> Self {
        self.orientation = orientation;
        self
    }
    #[allow(unused)]
    pub fn force(mut self, force: Vec3) -> Self {
        self._force = force;
        self
    }
    #[allow(unused)]
    pub fn inv_mass(mut self, inv_mass: Float) -> Self {
        self._inv_mass = inv_mass;
        self
    }
    #[allow(unused)]
    pub fn inertia(mut self, inertia: Vec3) -> Self {
        self._inertia = inertia;
        self
    }
    #[allow(unused)]
    pub fn restitution(mut self, restitution: Float) -> Self {
        self._restitution = restitution;
        self
    }
}