#[derive(Default)]
pub struct Timing{
    pub start_time: f64,
    pub sim_time: f64,
    pub physics_time_accumulator: f64,
    pub physics_time: f64,
    pub render_time_accumulator: f64,
    pub render_time: f64,
    pub frame_count: u32,
    pub fps: f64,
    pub dt: f64,
    pub runtime: f64,
}