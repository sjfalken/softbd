
mod ui;
mod softbody;
mod common;
mod input;
mod player;
mod movement;
mod constr;

use std::slice::Windows;
use constr::*;
use avian2d::dynamics::solver::schedule::SubstepSolverSet;
use movement::*;
use ui::*;
use softbody::*;
use common::*;
use input::*;
use player::*;

use avian2d::dynamics::solver::xpbd::solve_constraint;
use avian2d::math::Vector3;
use avian2d::prelude::*;
use bevy::{
    prelude::*, render::{
        mesh::VertexAttributeValues,
    }
};
use bevy::input::gamepad;
use bevy::window::{PrimaryWindow, WindowEvent, WindowResized};
use serde::{Serialize, Deserialize};
use wasm_bindgen::prelude::*;


const RES_WIDTH: u32 = 100;

const RES_HEIGHT: u32 = 100;


#[derive(Serialize, Deserialize)]
struct AppConfig {
    canvas_id: String,
    assets_path: String,
}

#[derive(Event)]
enum SceneEvent {
    SceneDirty
}

enum WallLocation {
    Left,
    Right,
    Top,
    Bottom,
}

#[derive(Component)]
struct SceneWall(WallLocation);

#[wasm_bindgen]
pub fn run(config: JsValue) {
    let mut app = App::new();
    let config: AppConfig = serde_wasm_bindgen::from_value(config).expect("invalid config value");

    let default_plugins =
        DefaultPlugins
            .set(ImagePlugin::default_nearest());
    let default_plugins = default_plugins
            .set(AssetPlugin {file_path: config.assets_path, ..default()})
            .set(WindowPlugin {
                primary_window:
                Some(Window {
                    canvas: Some(format!("#{}", config.canvas_id)),
                    fit_canvas_to_parent: true,
                    resizable: true,
                    ..default()
                }),
                ..default() });
    // #[cfg(not(target_arch = "wasm32"))]
    // let default_plugins = default_plugins
    //     .set(WindowPlugin {
    //         primary_window:
    //         Some(Window {
    //             resizable: false,
    //             ..default()
    //         }),
    //         ..default() });
    
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
                     (setup_ui, setup_scene, setup_softbody, setup_input, setup_player, setup_finish).chain())
        .add_systems(OnEnter(GameState::InGame),
                     (setup_joints,).chain())
        .add_systems(Update,
                     (
                         update_window,
                         update_ui, 
                         update_input, 
                         update_player,
                         update_softbody,
                         update_scene,
                         update_mesh, 
                         update_movement,
                     ).chain().run_if(in_state(GameState::InGame)))
        .init_state::<GameState>()
        .add_event::<SceneEvent>()
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
    let circle = Circle::new(5.0);

    let rect = Rectangle::new(1., 1.);
    
    let bot = (Transform::from_translation(Vector3::ZERO), SceneWall(WallLocation::Bottom));
    let top = (Transform::from_translation(Vector3::ZERO), SceneWall(WallLocation::Top));
    let left = (Transform::from_translation(Vector3::ZERO), SceneWall(WallLocation::Left));
    let right = (Transform::from_translation(Vector3::ZERO), SceneWall(WallLocation::Right));

    let rect_mesh = meshes.add(rect);
    
    for t in [bot, top, left, right] {
        commands.spawn((
            t,
            MeshMaterial2d(materials.add(ColorMaterial::from_color(Color::linear_rgb(1.0, 0., 0.)))),
            RigidBody::Static,
            rect.collider(),
            Mesh2d(rect_mesh.clone()),
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

fn update_window(
    mut projection: Single<&mut OrthographicProjection, With<OuterCamera>>,
    // mut events: EventReader<WindowEvent>,
    mut event_writer: EventWriter<SceneEvent>,
    window: Single<&Window, With<PrimaryWindow>>,
    mut uiscale: ResMut<UiScale>,
) {
    let h_scale = window.width() / RES_WIDTH as f32;
    let v_scale = window.height() / RES_HEIGHT as f32;

    let new_proj = 1. / h_scale.min(v_scale).round();
    
    if (projection.scale - new_proj).abs() > 0.001 {
        event_writer.send(SceneEvent::SceneDirty);
        info!("Scene is dirty, updating");
    }
    
    projection.scale = new_proj;
    uiscale.0 = 6. * h_scale.min(v_scale);
    
    
    
}

fn update_scene(
    mut events: EventReader<SceneEvent>,
    window: Single<&Window, With<PrimaryWindow>>,
    mut ball: Single<&mut Transform, With<SoftBody>>,
    mut walls: Query<(&mut Collider, &mut Transform, &SceneWall), Without<SoftBody>>,
) {
    for e in events.read() {
        match e {
            SceneEvent::SceneDirty => {
                let h_scale = window.width() / RES_WIDTH as f32;
                let v_scale = window.height() / RES_HEIGHT as f32;
                let ratio = h_scale / v_scale;
                
                let scene_width = RES_WIDTH as f32 * if h_scale < v_scale {
                    1.
                } else {
                    h_scale / v_scale
                };
                
                let scene_height = RES_HEIGHT as f32 * if h_scale < v_scale {
                    v_scale / h_scale
                } else {
                    1.
                };
                
                for (mut collider, mut transform, scene_wall) in walls.iter_mut() {
                    match scene_wall.0 {
                        WallLocation::Top => {
                            transform.translation = scene_height / 2. * Vec3::Y;
                            transform.scale = Vec3::new(scene_width, 10., 1.);
                            collider.set_scale(Vec2::new(scene_width, 10.), 2);
                        },
                        WallLocation::Bottom => {
                            transform.translation = -scene_height / 2. * Vec3::Y;
                            transform.scale = Vec3::new(scene_width, 10., 1.);
                            collider.set_scale(Vec2::new(scene_width, 10.), 2);
                        },
                        WallLocation::Left => {
                            transform.translation = -scene_width / 2. * Vec3::X;
                            transform.scale = Vec3::new(10., scene_height, 1.);
                            collider.set_scale(Vec2::new(10., scene_height), 2);
                        },
                        WallLocation::Right => {
                            transform.translation = scene_width / 2. * Vec3::X;
                            transform.scale = Vec3::new(10., scene_height, 1.);
                            collider.set_scale(Vec2::new(10., scene_height), 2);
                        },
                    }
                }

                ball.translation = Vec3::ZERO;
            }
        }
    }
}
