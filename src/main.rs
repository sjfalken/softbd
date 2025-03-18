//! Shows how to create graphics that snap to the pixel grid by rendering to a texture in 2D

mod ui;
mod softbody;
mod common;
mod input;
mod player;
mod movement;
mod constr;

use constr::*;
use avian2d::dynamics::solver::schedule::SubstepSolverSet;
use movement::*;
use ui::*;
use softbody::*;
use common::*;
use input::*;
use player::*;

use avian2d::dynamics::solver::xpbd::solve_constraint;
use avian2d::parry::simba::scalar::SupersetOf;
use avian2d::prelude::*;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy::{
    prelude::*, render::{
        mesh::VertexAttributeValues,
    }, window::WindowResized
};
use bevy::ecs::schedule::ScheduleLabel;
// use crate::constr::{iterative_constraint_solve, TriAreaConstraint};

/// In-game resolution width.
const RES_WIDTH: u32 = 160;

/// In-game resolution height.
const RES_HEIGHT: u32 = 90;

macro_rules! repeat_system {
    ( $z:expr, $count:expr ) => {{
        // ($z, repeat_system!($x)).chain()
        let mut result = $z;
        
        for _ in 1..$count {
            result = (result, $z).chain() 
        }
        
        result
    }};
}
fn main() {
    let mut app = App::new();
    app
        .add_plugins(
            DefaultPlugins
            .set(ImagePlugin::default_nearest())
        )
        .add_plugins(WorldInspectorPlugin::default())
        .add_plugins(PhysicsPlugins::default()
            .with_length_unit(10.)
            // .set(SolverSchedulePlugin {
            //
            // })
        )
        // .add_plugins(PhysicsDebugPlugin::default())
        .add_systems(Startup, start_loading)
        .add_systems(Update,
                     (update_loading).run_if(in_state(GameState::Loading)))
        .add_systems(OnEnter(GameState::Setup),
                     (setup_ui, setup_softbody, setup_scene, setup_input, setup_player, setup_finish).chain())
        .add_systems(OnEnter(GameState::InGame),
                     (setup_joints,).chain())
        .add_systems(Update,
                     (fit_canvas, update_ui, update_input, update_player, update_mesh, update_movement).chain().run_if(in_state(GameState::InGame)))
        .init_state::<GameState>()
        .insert_resource(Gravity(Vec2::NEG_Y * 100.0));
        // .run();

    app
        .get_schedule_mut(SubstepSchedule)
        .unwrap()
        // .configure_sets(
        //     (
        //
        //         )
        // )
        // .configure_sets(
        //     (
        //         Integration
        //     ).chain()
        // )
        // .configure_sets(
        //
        // )
        .add_systems(
            (
                iterative_constraint_begin::<TriAreaConstraint, 3>,
                iterative_constraint_begin::<EdgeDistanceConstraint, 2>,
                repeat_system!(
                    (
                        iterative_constraint_solve::<TriAreaConstraint, 3>,
                        iterative_constraint_solve::<EdgeDistanceConstraint, 2>,
                    ).chain(),
                    1
                )
            ).chain().in_set(SubstepSolverSet::SolveUserConstraints)
        )
        ;

    // app.add_schedule()
    // app.init_schedule(IterationSchedule);


    app.run();
    
}


// trait RepeatingSystem<M>: IntoSystemConfigs<M> {
//     fn repeat(self, num: usize) -> Self {
//     }
// }




#[derive(SystemSet, Clone, Hash, PartialEq, Eq, Debug)]
enum IterationSolverSet {
    SolveDistance,
    SolveArea,
}

#[derive(Debug, Hash, PartialEq, Eq, Clone, ScheduleLabel)]
struct IterationSchedule;

#[derive(Component)]
struct OuterCamera;




#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, Default, States)]
enum GameState {
    #[default]
    Loading,
    Setup,
    InGame,

}


fn start_loading(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    // mut state: ResMut<NextState<GameState>>,
) {
    info!("start_loading");
    let x = asset_server.load("gltf/circle.glb");
    commands.insert_resource(GameScene(x));

}



fn setup_finish(
    mut next_state: ResMut<NextState<GameState>>
) {
    next_state.set(GameState::InGame);
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let rect = Rectangle::new(160.0, 10.0);// .mesh().build();
    let circle = Circle::new(5.0);// .mesh().build();
    
    let bot = Transform::from_translation(-40. * Vec3::Y);
    let top = Transform::from_translation(40. * Vec3::Y);
    let left = Transform::from_translation(-75.* Vec3::X).with_rotation(Quat::from(Rotation::degrees(90.)));
    let right = Transform::from_translation(75.* Vec3::X).with_rotation(Quat::from(Rotation::degrees(90.)));

    for t in [bot, top, left, right] {
        commands.spawn((
            t,
            MeshMaterial2d(materials.add(ColorMaterial::from_color(Color::linear_rgb(1.0, 0., 0.)))),
            RigidBody::Static,
            rect.collider(),
            Mesh2d(meshes.add(rect)),
            CollisionLayers::new(GameLayer::default(), GameLayer::all_bits()),
            // Collider::rectangle(100., 10.),
        ));
    }
    commands.spawn((
        Transform::from_translation(Vec3::new(40.0, 10.0, 0.0)),
        MeshMaterial2d(materials.add(ColorMaterial::from_color(Color::linear_rgb(1.0, 0., 0.)))),
        RigidBody::Dynamic,

        circle.collider(),
        Mesh2d(meshes.add(circle)),
        // CollisionLayers::new(GameLayer::default(), GameLayer::default()),
        // Collider::rectangle(100., 10.),
    ));

    commands.spawn((
        Camera2d::default(),
        OuterCamera,
    ));
}



// fn setup_sim(mut commands: Commands) {
// }


fn update_mesh(
    mesh: Single<&mut Mesh2d, With<SoftBody>>,
    mut meshes: ResMut<Assets<Mesh>>,
    points: Query<Entity, With<SoftBodyPoint>>,
    point_transforms: Query<&Transform, With<SoftBodyPoint>>,
    point_vertex_indices: Query<&VertexIndex, With<SoftBodyPoint>>,


    // TODO remove the mesh_indices from the point entirely, as its only needed for the initial setup in setup_mesh()
    // point_mesh_indices: Query<&MeshIndices, With<SoftBody>>,
) {

    let mesh_data = meshes.get_mut(mesh.id()).unwrap();
    let Some(VertexAttributeValues::Float32x3(positions)) = mesh_data.attribute_mut(Mesh::ATTRIBUTE_POSITION) else {
        panic!("can't extract positions");
    };


    points.iter().for_each(| pe | {
        let vertex_index = point_vertex_indices.get(pe).unwrap().0;
        let transform = point_transforms.get(pe).unwrap();

        positions[vertex_index as usize] = transform.translation.to_array();
    });
}
fn update_loading(
    // mut commands: Commands,
    asset_server: Res<AssetServer>,
    scene: Res<GameScene>,
    mut next_state: ResMut<NextState<GameState>>
) {
    // info!("update_loading");
    let state = asset_server.load_state(scene.id());
    // info!("{state:?}");

    if asset_server.is_loaded(scene.id()) {
        info!("Loaded scene");


        next_state.set(GameState::Setup);
    }

}

fn fit_canvas(
    mut resize_events: EventReader<WindowResized>,
    mut projection: Single<&mut OrthographicProjection, With<OuterCamera>>,
) {
    for event in resize_events.read() {
        let h_scale = event.width / RES_WIDTH as f32;
        let v_scale = event.height / RES_HEIGHT as f32;
        projection.scale = 1. / h_scale.min(v_scale).round();
    }
}
