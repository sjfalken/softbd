use avian2d::math::Vector;
use avian2d::prelude::*;
use bevy::prelude::*;
use crate::common::*;
use crate::player::PlayerState;

pub fn setup_movement (
    mut commands: Commands,
) {

}

pub fn update_movement (
    player_state: Single<&PlayerState>,
    mut point_forces: Query<&mut ExternalForce, With<SoftBodyPoint>>,
    // mut point_forces: Query<&mut mass, With<SoftBodyPoint>>
    time: Res<Time>,
) {
    
    // info!("update_movement");
    if (player_state.is_jumping) {
        point_forces.iter_mut().for_each(|mut force| {
            // info!("{force:?}");
            force.apply_force(Vector::new(0., 20.));
        })
    } else {
    }
}