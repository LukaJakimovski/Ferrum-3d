use egui_wgpu::wgpu;
use ferrum_core::time::now;
use crate::app::App;

impl App {
    pub fn update(&mut self) {
        let state = match &mut self.state {
            Some(canvas) => canvas,
            None => return,
        };
        
        let dt = now() - self.last_time;
        self.last_time = now();
        
        state.update(dt);
        Self::render_update(state);
    }
    
    
    pub fn render_update(state: &mut ferrum_render::State){
        match state.render() {
            Ok(_) => {}
            // Reconfigure the surface if it's lost or outdated
            Err(wgpu::SurfaceError::Lost | wgpu::SurfaceError::Outdated) => {
                let size = state.window.inner_size();
                state.resize(size.width, size.height);
            }
            Err(e) => {
                log::error!("Unable to render {}", e);
            }
        }
    }
}