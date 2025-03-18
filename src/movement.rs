use avian2d::math::Vector;
use avian2d::prelude::*;
use bevy::prelude::*;
use crate::common::*;
use crate::player::{PlayerConfig, PlayerState};

pub fn setup_movement (
    mut commands: Commands,
) {

}

pub fn update_movement (
    player: Single<(&PlayerState, &PlayerConfig)>,
    mut points: Query<(&mut ExternalForce, &Mass), With<SoftBodyPoint>>,
    // mut point_forces: Query<&mut mass, With<SoftBodyPoint>>
    time: Res<Time>,
    grav: Res<Gravity>
) {
    let (state, config) = player.into_inner();
    
    let grav_acc = grav.into_inner().0;
    if state.is_jumping {
        points.iter_mut().for_each(|(mut force, mass)| {
            let desired_accel = -2. * grav_acc;
            let needed_force = desired_accel * mass.0;
            
            // is cleared every frame
            force.apply_force(needed_force);
        })
    } else {
    }
}