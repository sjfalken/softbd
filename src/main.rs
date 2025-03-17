//! Shows how to create graphics that snap to the pixel grid by rendering to a texture in 2D

mod ui;
mod softbody;
mod common;
mod input;
mod player;
mod movement;

use movement::*;
use ui::*;
use softbody::*;
use common::*;
use input::*;
use player::*;

use avian2d::prelude::*;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy::{
    prelude::*, render::{
        mesh::VertexAttributeValues,
    }, window::WindowResized
};

/// In-game resolution width.
const RES_WIDTH: u32 = 160;

/// In-game resolution height.
const RES_HEIGHT: u32 = 90;

fn main() {
    App::new()
        .add_plugins(
            DefaultPlugins
            .set(ImagePlugin::default_nearest())
        )
        .add_plugins(WorldInspectorPlugin::default())
        .add_plugins(PhysicsPlugins::default()
            .with_length_unit(10.)
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
        // .insert_resource(Gravity(Vec2::NEG_Y * 100.0))
        .run();
    
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
