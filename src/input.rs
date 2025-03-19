

use bevy::input::ButtonInput;
use bevy::prelude::*;

#[derive(Resource)]
pub struct PlayerInput {
    pub do_jump: bool,
    // jump_elapsed: f32,
    
}
impl PlayerInput {
    fn new() -> Self {
        Self {
            do_jump: false,
            // jump_elapsed: 0.,
        }
    }
}

pub fn setup_input(mut commands: Commands) {
    commands.insert_resource(PlayerInput::new());
}

pub fn update_input(
    mut player_input: ResMut<PlayerInput>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    mut touches: ResMut<Touches>
) {


    let touch_count = touches.iter().count();
    
    if keyboard.pressed(KeyCode::Space) || touch_count > 0 || mouse.pressed(MouseButton::Left) {
        player_input.do_jump = true;
    } else {
        player_input.do_jump = false;
    }
    
    touches.clear();
}
    

