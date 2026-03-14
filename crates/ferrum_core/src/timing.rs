#[derive(Default)]
pub struct Timing{
    pub start_time: f64,
    pub frame_count: u32,
    pub timer: f64,
    pub fps: f64,
    pub runtime: f64,
    pub test: bool,
}