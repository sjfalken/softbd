

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
    keyboard: Res<ButtonInput<KeyCode>>
) {
    // player_input.jump_elapsed += time.delta_secs();
    if keyboard.pressed(KeyCode::Space) {
        player_input.do_jump = true;
    } else {
        player_input.do_jump = false;
    }
}
    

