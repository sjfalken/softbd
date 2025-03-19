
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
use avian2d::prelude::*;
use bevy::{
    prelude::*, render::{
        mesh::VertexAttributeValues,
    }
};
use bevy::ecs::schedule::ScheduleLabel;
use wasm_bindgen::prelude::*;

const RES_WIDTH: u32 = 160;

const RES_HEIGHT: u32 = 90;

#[wasm_bindgen]
pub fn run() {
    let mut app = App::new();

    let default_plugins =
        DefaultPlugins
            .set(ImagePlugin::default_nearest());
    #[cfg(target_arch = "wasm32")]
    let default_plugins = default_plugins
            .set(AssetPlugin {file_path: String::from("/assets"), ..default()})
            .set(WindowPlugin {
                primary_window:
                Some(Window {
                    canvas: Some(String::from("#mycanvas")),
                    fit_canvas_to_parent: true,
                    resizable: false,
                    ..default()
                }),
                ..default() });
    #[cfg(not(target_arch = "wasm32"))]
    let default_plugins = default_plugins
        .set(WindowPlugin {
            primary_window:
            Some(Window {
                resizable: false,
                ..default()
            }),
            ..default() });
    
    app
        .add_plugins(
            default_plugins
        )
        .add_plugins(PhysicsPlugins::default()
            .with_length_unit(10.)
        )
        // .add_plugins(PhysicsDebugPlugin::default())
        .add_systems(Startup, start_loading)
        .add_systems(Update,
                     (update_loading).run_if(in_state(GameState::Loading)))
        .add_systems(OnEnter(GameState::Setup),
                     (setup_ui, setup_scene, fit_canvas, setup_softbody, setup_input, setup_player, setup_finish).chain())
        .add_systems(OnEnter(GameState::InGame),
                     (setup_joints,).chain())
        .add_systems(Update,
                     (fit_canvas, update_ui, update_input, update_player, update_softbody, update_mesh, update_movement).chain().run_if(in_state(GameState::InGame)))
        .init_state::<GameState>()
        .insert_resource(Gravity(Vec2::NEG_Y * 100.0));

    app
        .get_schedule_mut(SubstepSchedule)
        .unwrap()

        .add_systems(
            (

                solve_constraint::<TriAreaConstraint, 3>
            ).chain().in_set(SubstepSolverSet::SolveUserConstraints)
        )
    ;

    app.run();

}

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
    let rect = Rectangle::new(160.0, 20.0);
    let circle = Circle::new(5.0);

    let bot = Transform::from_translation(-45. * Vec3::Y);
    let top = Transform::from_translation(45. * Vec3::Y);
    let left = Transform::from_translation(-80.* Vec3::X).with_rotation(Quat::from(Rotation::degrees(90.)));
    let right = Transform::from_translation(80.* Vec3::X).with_rotation(Quat::from(Rotation::degrees(90.)));

    for t in [bot, top, left, right] {
        commands.spawn((
            t,
            MeshMaterial2d(materials.add(ColorMaterial::from_color(Color::linear_rgb(1.0, 0., 0.)))),
            RigidBody::Static,
            rect.collider(),
            Mesh2d(meshes.add(rect)),
            CollisionLayers::new(GameLayer::default(), GameLayer::all_bits()),
        ));
    }
    commands.spawn((
        Transform::from_translation(Vec3::new(40.0, 10.0, 0.0)),
        MeshMaterial2d(materials.add(ColorMaterial::from_color(Color::linear_rgb(1.0, 0., 0.)))),
        RigidBody::Dynamic,

        circle.collider(),
        Mesh2d(meshes.add(circle)),
    ));

    commands.spawn((
        Camera2d::default(),
        OuterCamera,
    ));
}



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
    asset_server: Res<AssetServer>,
    scene: Res<GameScene>,
    mut next_state: ResMut<NextState<GameState>>
) {
    let state = asset_server.load_state(scene.id());

    if asset_server.is_loaded(scene.id()) {
        info!("Loaded scene");


        next_state.set(GameState::Setup);
    }

}

fn fit_canvas(
    mut projection: Single<&mut OrthographicProjection, With<OuterCamera>>,
    window: Single<&Window>,
    mut uiscale: ResMut<UiScale>,
) {
    let h_scale = window.width() / RES_WIDTH as f32;
    let v_scale = window.height() / RES_HEIGHT as f32;
    projection.scale = 1. / h_scale.min(v_scale).round();
    uiscale.0 = 6. * v_scale;
}
