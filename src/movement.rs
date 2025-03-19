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
    mut points: Query<(&mut ExternalImpulse, &ComputedMass), With<SoftBodyPoint>>,
    // mut point_forces: Query<&mut mass, With<SoftBodyPoint>>
    time: Res<Time>,
    grav: Res<Gravity>
) {
    let (state, config) = player.into_inner();
    
    let grav_acc = grav.into_inner().0;
    if state.is_jumping {
        points.iter_mut().for_each(|(mut impulse, mass)| {
            let desired_accel = -4. * grav_acc;
            let needed_force = desired_accel * (1./mass.inverse());
            let needed_impulse = needed_force * time.delta_secs();
            
            impulse.apply_impulse(needed_impulse);
            
        })
    } else {
    }
}