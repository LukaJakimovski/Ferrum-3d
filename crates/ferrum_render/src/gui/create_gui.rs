use egui::Align2;
use egui_wgpu::{wgpu, ScreenDescriptor};
use crate::State;

#[repr(usize)]
#[derive(Debug, Copy, Clone)]
pub enum Menu {
    Config = 0,
    Timer = 1,
    Energy = 2,
    Camera = 3,
    Spawner = 4,
    Input = 5,
    Editor = 6,
    DragParams = 7,
    Advanced = 8,
    Color = 10,
}

impl State {
    pub fn create_gui(&mut self, encoder: &mut wgpu::CommandEncoder, view: &wgpu::TextureView){
        let screen_descriptor = ScreenDescriptor {
            size_in_pixels: [self.config.width, self.config.height],
            pixels_per_point: self.window.as_ref().scale_factor() as f32 * self.config.height as f32 / 1440.0,
        };
        self.egui_renderer.begin_frame(self.window.as_ref());

        let renderer = self.egui_renderer.context();
        let menus = &mut self.menus;
        egui::Window::new("Menu Selector")
            .resizable(false)
            .vscroll(false)
            .anchor(Align2::LEFT_TOP, [0.0, 0.0])
            .default_open(true)
            .title_bar(false)
            .show(&renderer, |ui| {
                ui.heading("Menu Selector");
                ui.checkbox(&mut menus[Menu::Energy as usize], "Energy Info", );
                ui.checkbox(&mut menus[Menu::Timer as usize], "Timing Info", );
            });
        let menus = &self.menus;

        if menus[Menu::Energy as usize] {
            self.energy_menu();
        }
        if menus[Menu::Timer as usize] {
            self.timing_menu();
        }

        self.egui_renderer.end_frame_and_draw(
            &self.device,
            &self.queue,
            encoder,
            self.window.as_ref(),
            &view,
            screen_descriptor,
        );
    }

    fn energy_menu(&self) {
        let renderer = self.egui_renderer.context();
        let energy = &self.physics.energy;
        egui::Window::new("Energy")
            .resizable(false)
            .vscroll(true)
            .default_open(true)
            .max_height(200.0)
            .max_width(300.0)
            .title_bar(false)
            .show(renderer, |ui| {
                ui.heading("Energy");
                ui.label(format!("Kinetic Energy: {:.3} Joules", energy.kinetic_energy));
                ui.label(format!("Rotational Energy: {:.3} Joules", energy.rotational_kinetic_energy));
                ui.label(format!("Potential Energy: {:.3} Joules", energy.gravitational_potential_energy));
                ui.label(format!("Total Energy: {:.3} Joules", energy.total_energy));
                ui.label(format!("Delta Energy: {:.3} Joules", energy.total_energy - energy.start_energy));
            });
    }

    fn timing_menu(&self) {
        let renderer = self.egui_renderer.context();
        let timer = &self.timer;
        egui::Window::new("Timer")
            .resizable(false)
            .vscroll(true)
            .default_open(true)
            .max_height(200.0)
            .max_width(300.0)
            .title_bar(false)
            .show(renderer, |ui| {
                ui.heading("Timer");
                ui.label(format!("Runtime: {:.3}s", timer.runtime));
                ui.label(format!("Sim Time: {:.3}s", timer.sim_time));
                ui.label(format!("Ratio: {:.2}x", timer.runtime / timer.sim_time));
                ui.label(format!("Delta Time: {:.3}ms", timer.dt * 1000.0));
                ui.label(format!("Render FPS: {:.3}fps", timer.fps));

            });
    }
}