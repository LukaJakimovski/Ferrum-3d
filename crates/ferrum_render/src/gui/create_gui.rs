use egui::Align2;
use egui_wgpu::{wgpu, ScreenDescriptor};
use crate::State;

#[repr(usize)]
#[derive(Debug, Copy, Clone)]
pub enum Menu {
    Properties = 0,
    Timer = 1,
    Energy = 2,
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
                ui.checkbox(&mut menus[Menu::Properties as usize], "Properties", );
            });
        let menus = &self.menus;

        if menus[Menu::Energy as usize] {
            self.energy_menu();
        }
        if menus[Menu::Timer as usize] {
            self.timing_menu();
        }
        if menus[Menu::Properties as usize] {
            self.properties_menu();
        }

        if !self.mouse_pressed {
            self.is_pointer_used = self.egui_renderer.context().is_pointer_over_area();
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
                ui.label(format!("Render FPS: {:.1}fps", timer.fps));

            });
    }

    fn properties_menu(&mut self){
        let renderer = self.egui_renderer.context();
        let rigidbodies = &self.physics.rigidbodies;
        let i = &mut self.selected_index;
        egui::Window::new("Timer")
            .resizable(false)
            .vscroll(true)
            .default_open(true)
            .max_height(200.0)
            .max_width(400.0)
            .title_bar(false)
            .show(renderer, |ui| {
                ui.heading("Properties");
                ui.columns(2, |ui| {
                    ui[0].label("Index");
                    ui[1].add(egui::DragValue::new(i).speed(1));
                    if *i > (rigidbodies.len() - 1) {
                        *i = rigidbodies.len() - 1;
                    }
                });
                let i = *i;
                ui.label("Position");
                ui.columns(3, |ui| {
                    ui[0].label(format!("{:.3}m", rigidbodies.positions[i].x));
                    ui[1].label(format!("{:.3}m", rigidbodies.positions[i].y));
                    ui[2].label(format!("{:.3}m", rigidbodies.positions[i].z));
                });
                ui.label(format!("Velocity {}m/s", rigidbodies.velocities[i].length()));
                ui.columns(3, |ui| {
                    ui[0].label(format!("{:.3}m/s", rigidbodies.velocities[i].x));
                    ui[1].label(format!("{:.3}m/s", rigidbodies.velocities[i].y));
                    ui[2].label(format!("{:.3}m/s", rigidbodies.velocities[i].z));
                });
                ui.label(format!("Force {}N", rigidbodies.get_forces(i).length()));
                ui.columns(3, |ui| {
                    ui[0].label(format!("{:.3}N", rigidbodies.get_forces(i).x));
                    ui[1].label(format!("{:.3}N", rigidbodies.get_forces(i).y));
                    ui[2].label(format!("{:.3}N", rigidbodies.get_forces(i).z));
                });
                ui.label("Mass");
                ui.label(format!("{:.2}kg", rigidbodies.get_mass(i)));

                ui.label("Orientation");
                ui.columns(4, |ui| {
                    ui[0].label(format!("{:.3}", rigidbodies.get_orientation(i).x));
                    ui[1].label(format!("{:.3}x", rigidbodies.get_orientation(i).y));
                    ui[2].label(format!("{:.3}y", rigidbodies.get_orientation(i).z));
                    ui[3].label(format!("{:.3}z", rigidbodies.get_orientation(i).w));
                });
                ui.label(format!("Torque {}Nm", rigidbodies.get_torques(i).length()));
                ui.columns(3, |ui| {
                    ui[0].label(format!("{:.3}N", rigidbodies.get_torques(i).x));
                    ui[1].label(format!("{:.3}N", rigidbodies.get_torques(i).y));
                    ui[2].label(format!("{:.3}N", rigidbodies.get_torques(i).z));
                });
                ui.label("Inertia Tensor");
                let iner = rigidbodies.get_inertia(i).to_cols_array_2d();
                ui.columns(3, |ui| {
                    ui[0].label(format!("{:.3}kg*m^2", iner[0][0]));
                    ui[1].label(format!("{:.3}kg*m^2", iner[0][1]));
                    ui[2].label(format!("{:.3}kg*m^2", iner[0][2]));
                });
                ui.columns(3, |ui| {
                    ui[0].label(format!("{:.3}kg*m^2", iner[1][0]));
                    ui[1].label(format!("{:.3}kg*m^2", iner[1][1]));
                    ui[2].label(format!("{:.3}kg*m^2", iner[1][2]));
                });
                ui.columns(3, |ui| {
                    ui[0].label(format!("{:.3}kg*m^2", iner[2][0]));
                    ui[1].label(format!("{:.3}kg*m^2", iner[2][1]));
                    ui[2].label(format!("{:.3}kg*m^2", iner[2][2]));
                });
                ui.label("Kinetic Energy");
                ui.label(format!("{:.3}J", 0.5 * rigidbodies.get_mass(i) * rigidbodies.get_velocity(i).length_squared()));
                ui.label("Rotational Energy");
                ui.label(format!("{:.3}J", 0.5 * (rigidbodies.get_inertia(i) * rigidbodies.get_omega(i)).length_squared()));
            });
    }
}