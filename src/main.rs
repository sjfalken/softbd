//! Shows how to create graphics that snap to the pixel grid by rendering to a texture in 2D

mod ui;
mod softbody;
mod common;

use softbody::*;
use common::*;

use std::time::Duration;
use std::collections::{BTreeSet, HashSet};
use avian2d::dynamics::solver::SolverConfig;
use avian2d::prelude::*;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy::{
    asset::{RenderAssetUsages}, core::FrameCount, ecs::world, gltf::GltfMesh, log::{Level, LogPlugin}, math::NormedVectorSpace, prelude::*, render::{
        camera::RenderTarget, mesh::{CircleMeshBuilder, MeshVertexAttributeId, VertexAttributeValues}, render_resource::{
            Extent3d, TextureDescriptor, TextureDimension, TextureFormat, TextureUsages,
        }, view::RenderLayers
    }, sprite::Anchor, utils::hashbrown::HashMap, window::WindowResized
};
use bevy::render::mesh::{Indices, PrimitiveTopology};

/// In-game resolution width.
const RES_WIDTH: u32 = 160;

/// In-game resolution height.
const RES_HEIGHT: u32 = 90;

fn main() {
    App::new()
        .add_plugins(
            DefaultPlugins
            .set(ImagePlugin::default_nearest())
            // .set(AssetPlugin {
            //     file_path: String::from(std::env::var("ASSETS_PATH").unwrap()), 
            //     ..default()
            // })
        )
        .add_plugins(WorldInspectorPlugin::default())
        .add_plugins(PhysicsPlugins::default()
            .with_length_unit(50.)
        )
        // .add_plugins(PhysicsDebugPlugin::default())

        .add_systems(Startup, start_loading)
        .add_systems(Update, (update_loading).run_if(in_state(GameState::Loading)))
        .add_systems(OnEnter(GameState::Setup), (ui::setup_ui, setup_softbody, setup_scene, setup_finish).chain())
        .add_systems(OnEnter(GameState::InGame), (setup_joints,).chain())
        .add_systems(Update, (fit_canvas, update_mesh, ui::update_ui, update_input).run_if(in_state(GameState::InGame)))
        // .add_systems(Update, update_loading)
        .init_state::<GameState>()
        .run();
    
}

#[derive(Component)]
struct OuterCamera;


#[derive(Resource)]
struct PlayerInput {
    is_jumping: bool,
    jump_time: f32,
}

impl PlayerInput {
    fn new() -> Self {
        Self {
            is_jumping: false,
            jump_time: 0.,
        }
    }
    fn do_jump(&mut self, now: f32) {
        if (!self.is_jumping) {
            self.is_jumping = true;
            self.jump_time = now;
        }
    }
}


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
    let rect = Rectangle::new(100.0, 10.0);// .mesh().build();
    let circle = Circle::new(5.0);// .mesh().build();
    commands.spawn((
        Transform::from_translation(-30. * Vec3::Y),
        MeshMaterial2d(materials.add(ColorMaterial::from_color(Color::linear_rgb(1.0, 0., 0.)))),
        RigidBody::Static,
        rect.collider(),
        Mesh2d(meshes.add(rect)),
        CollisionLayers::new(GameLayer::default(), GameLayer::all_bits()),
        // Collider::rectangle(100., 10.),
    ));
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


fn update_input(keyboard: Res<ButtonInput<KeyCode>>) {
    if keyboard.just_pressed(KeyCode::Space) {
        
    }
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
/// Scales camera projection to fit the window (integer multiples only).
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
