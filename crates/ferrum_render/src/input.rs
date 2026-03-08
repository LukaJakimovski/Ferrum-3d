use winit::event::{MouseButton, MouseScrollDelta};
use winit::event_loop::ActiveEventLoop;
use winit::keyboard::KeyCode;
use crate::State;

impl State {
     // UPDATED!
     pub fn handle_key(&mut self, event_loop: &ActiveEventLoop, key: KeyCode, pressed: bool) {
         if !self.camera_controller.handle_key(key, pressed) {
             match (key, pressed) {
                 (KeyCode::Escape, true) => event_loop.exit(),
                 _ => {}
             }
         }
     }

     // NEW!
     pub fn handle_mouse_button(&mut self, button: MouseButton, pressed: bool) {
         match button {
             MouseButton::Left => self.mouse_pressed = pressed,
             _ => {}
         }
     }

     // NEW!
     pub fn handle_mouse_scroll(&mut self, delta: &MouseScrollDelta) {
         self.camera_controller.handle_scroll(delta);
     }
 }