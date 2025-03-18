
use bevy::prelude::*;
use crate::input::PlayerInput;

#[derive(Component)]
pub struct PlayerState {
    pub is_jumping: bool,
    pub jump_time: f32,
    pub is_airborne: bool,
}


#[derive(Component)]
pub struct PlayerConfig {
    max_jump_time: f32,
}

impl PlayerConfig {
    pub fn max_jump_time(&self) -> f32 { self.max_jump_time }
}

pub fn setup_player(mut commands: Commands) {
    commands.spawn((
        PlayerState {
            is_airborne: false,
            is_jumping: false,
            jump_time: 0.0,
        },
        PlayerConfig {
            max_jump_time: 0.2
        }
    ));
}

pub fn update_player(
    mut player_state: Single<&mut PlayerState>,
    player_input: Res<PlayerInput>,
    time: Res<Time>,
) {
    if !player_state.is_airborne &&  !player_state.is_jumping &&  player_input.do_jump {
        info!("starting jump");
        player_state.is_jumping = true; 
    }
    
    if player_state.is_jumping {
        player_state.jump_time += time.delta_secs();
    }

    if player_state.jump_time > 0.2 {
        info!("stopping jump");
        player_state.jump_time = 0.0;
        player_state.is_jumping = false;
    }
}