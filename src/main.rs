//! Shows how to create graphics that snap to the pixel grid by rendering to a texture in 2D

use std::{ops::DerefMut, time::Duration};
use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet, HashSet};
// use avian2d::prelude::*;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy::{
    asset::{LoadedAsset, RenderAssetUsages}, core::FrameCount, ecs::world, gltf::GltfMesh, log::{Level, LogPlugin}, math::NormedVectorSpace, prelude::*, render::{
        camera::RenderTarget, mesh::{CircleMeshBuilder, MeshVertexAttributeId, VertexAttributeValues}, render_resource::{
            Extent3d, TextureDescriptor, TextureDimension, TextureFormat, TextureUsages,
        }, view::RenderLayers
    }, sprite::Anchor, utils::hashbrown::HashMap, window::WindowResized
};
use bevy::render::mesh::{Indices, MeshVertexAttribute, PrimitiveTopology};
// use std::en

/// In-game resolution width.
const RES_WIDTH: u32 = 160;

/// In-game resolution height.
const RES_HEIGHT: u32 = 90;

/// Default render layers for pixel-perfect rendering.
/// You can skip adding this component, as this is the default.
const PIXEL_PERFECT_LAYERS: RenderLayers = RenderLayers::layer(0);

/// Render layers for high-resolution rendering.
const HIGH_RES_LAYERS: RenderLayers = RenderLayers::layer(1);



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

        .add_systems(Startup, start_loading)
        .add_systems(OnEnter(GameState::InGame), (setup_camera, setup_ui, setup_mesh).chain())
        .add_systems(Update, (fit_canvas, update_soft, update_ui).run_if(in_state(GameState::InGame)))
        .add_systems(Update, (update_loading).run_if(in_state(GameState::Loading)))
        // .add_systems(Update, update_loading)
        .init_state::<GameState>()
        .run();
    
}

/// Low-resolution texture that contains the pixel-perfect world.
/// Canvas itself is rendered to the high-resolution world.
#[derive(Component)]
struct Canvas;

/// Camera that renders the pixel-perfect world to the [`Canvas`].
#[derive(Component)]
struct InGameCamera;

/// Camera that renders the [`Canvas`] (and other graphics on [`HIGH_RES_LAYERS`]) to the screen.
#[derive(Component)]
struct OuterCamera;

#[derive(Component)]
struct FPSNode;

struct SimDistanceConstraint {
    target_point_idx: usize,
    rest_distance: f32
}

struct SimPoint {
    // vert: SimVertex,
    // tri_membership: Vec<usize>,
    x: Vec2,
    v: Vec2,
    m: f32,
    
    mesh_indices: Vec<u16>,
    distance_constr: Vec<SimDistanceConstraint>
}


const VERTEX_EPS: f32 = 0.005;
#[derive(Deref, DerefMut)]
struct SimVertex(Vec3);

impl Eq for SimVertex {}

impl PartialEq<Self> for SimVertex {
    fn eq(&self, other: &Self) -> bool {
        return (self[0] - other[0]).abs() < VERTEX_EPS
            && (self[1] - other[1]).abs() < VERTEX_EPS
            && (self[2] - other[2]).abs() < VERTEX_EPS;
    }
}

// impl Ord for SimVertex {
//     fn cmp(&self, other: &Self) -> Ordering {
//         self.partial_cmp(other).expect("invalid comparison for vertex")
//     }
// }

// ordered by dimension 0, then 1, then 2
// impl PartialOrd for SimVertex {
//     fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
//         if (self[0] - other[0]).abs() < VERTEX_EPS {
//             if (self[1] - other[1]).abs() < VERTEX_EPS {
//                 if (self[2] - other[2]).abs() < VERTEX_EPS {
//                     Some(Ordering::Equal)
//                 } else {
//                     self[2].partial_cmp(&other[2])
//                 }
//             } else {
//                 self[1].partial_cmp(&other[1])
//             }
//         } else {
//             self[0].partial_cmp(&other[0])
//         }
//     }
// }

#[derive(Resource)]
struct SimData {
    template_mesh: Mesh,
    points: Vec<SimPoint>,
}

impl SimData {
    fn update(&mut self) {
        // p.vert.0 += 0.1 * Vec3::Y;
        let Some(VertexAttributeValues::Float32x3(positions)) = self.template_mesh.attribute_mut(Mesh::ATTRIBUTE_POSITION) else { 
            panic!("no position");
        };
        
        
        for (i, p) in positions.iter_mut().enumerate() {
            // p[1] += 0.05;
            p[0] = self.points[i].x[0];
            p[1] = self.points[i].x[1];
        }
    }
    // fn to_mesh(&self) ->  &Mesh {
    //     &self._template_mesh
    // }

    fn from_mesh(m: &Mesh) -> Self {

        let VertexAttributeValues::Float32x3(positions) = m.attribute(Mesh::ATTRIBUTE_POSITION).unwrap() else {
            panic!("error: not vec3");
        };

        let mut vertices = positions.clone().iter().enumerate().map(|(i, v)| {
            (SimVertex(Vec3::new(v[0], v[1], v[2])), vec![i])
        }).collect::<Vec<(SimVertex, Vec<usize>)>>();

        let mut start = 0;
        loop {
            if start >= vertices.len() {
                break;
            }
            
            let (v0, t0) = &vertices[start];
            

            let mut to_rm = Vec::new();

            for j in (start+1)..vertices.len() {
                if &vertices[j].0 == v0 {
                    to_rm.push(j);
                }
            }

            for i in (0..to_rm.len()).rev() {
                let t = vertices[to_rm[i]].1.clone();
                vertices[start].1.extend(t);
                vertices.remove(to_rm[i]);
            }

            start += 1;
        }

        let points_with_orig_idx = vertices;
        
        let mut template_mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::RENDER_WORLD | RenderAssetUsages::MAIN_WORLD);
        let mut indices: Vec<u16> = Vec::new();
        indices.resize(positions.len(), 0);
        
        let mut positions: Vec<[f32; 3]> = Vec::new();
        
        points_with_orig_idx.iter().enumerate().for_each(|(i, (p, orig))| {
            positions.push(p.0.to_array());
            for x in orig {
                indices[*x] = i as u16;
            }
        });
        template_mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, VertexAttributeValues::Float32x3(positions));
        template_mesh.insert_indices(Indices::U16(indices));
        
        // let mut tri_membership: Vec<Vec<usize>> = Vec::new();
        let mut points = points_with_orig_idx.iter().enumerate().map(|(i, (p, orig))| {
            // tri_membership.push(orig.iter().map(|x| x / 3).collect());
            SimPoint {
                x: Vec2::new(p[0], p[1]), 
                v: Vec2::ZERO, 
                m: 1., 
                mesh_indices: orig.iter().map(|x| {*x as u16}).collect(),
                distance_constr: Vec::new(),
            }
        }).collect::<Vec<SimPoint>>();

        (0..points.len()).for_each(|i| {

            (0..points.len()).for_each(|j| {
                if i == j {
                    return;
                }
                let t1: Vec<usize> = points[i].mesh_indices.iter().map(|x| *x as usize / 3).collect();
                let t2: Vec<usize> = points[j].mesh_indices.iter().map(|x| *x as usize / 3).collect();
                
                for t in t1 {
                    if t2.contains(&t) {
                        
                        let p1 = points[i].x;
                        let p2 = points[j].x;

                        points[i].distance_constr.push(
                            SimDistanceConstraint { 
                                rest_distance: (p2 - p1).norm(),
                                target_point_idx: j,
                            });
                    }
                }
            });
        });
        
        Self {
            points,
            template_mesh: template_mesh,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, Default, States)]
enum GameState {
    #[default]
    Loading,
    InGame,
}
// fn setup_time(mut ) {
//     // the sample sprite that will be rendered to the pixel-perfect canvas
//     // commands.spawn((
//     //     Sprite::from_image(asset_server.load("pixel/bevy_pixel_dark.png")),
//     //     Transform::from_xyz(-40., 20., 2.),
//     //     Rotate,
//     //     PIXEL_PERFECT_LAYERS,
//     // ));

//     // // the sample sprite that will be rendered to the high-res "outer world"
//     // commands.spawn((
//     //     Sprite::from_image(asset_server.load("pixel/bevy_pixel_light.png")),
//     //     Transform::from_xyz(-40., -20., 2.),
//     //     Rotate,
//     //     HIGH_RES_LAYERS,
//     // ));
// }
#[derive(Resource, Deref, DerefMut)]
struct MyTimer(Timer);


#[derive(Resource, Deref)]
struct MyScene(Handle<Gltf>);

impl MyTimer {
    // type Target = Timer;
    fn new() -> Self {
        MyTimer (Timer::from_seconds(1., TimerMode::Repeating))
    }

    // fn tick(&mut self, d : Duration) {
    //     self.0.tick(d);
    // }
}

// impl DerefMut for MyTimer {
//     fn deref_mut(&mut self) -> &mut Self::Target {
        
//     }
// }

fn setup_ui(mut commands: Commands) {
    commands.insert_resource(MyTimer::new());
    commands.spawn((
        Node {
            display: Display::Flex, 
            flex_direction: FlexDirection::Row,
            width: Val::Percent(100.), ..default()
        },
        // HIGH_RES_LAYERS,
    ))
    .with_children(
        |builder| {
            builder.spawn((
                Text::new("0"), 
                FPSNode,
                TextFont { 
                    font_size: 20.0, 
                    ..default() 
                },
                TextColor::WHITE,
                // HIGH_RES_LAYERS
                
            ));
        }
    );
}

fn start_loading(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    // mut state: ResMut<NextState<GameState>>,
) {
    info!("start_loading");
    let x = asset_server.load("gltf/circle.glb");
    commands.insert_resource(MyScene(x));

    // let s = state.into_inner().get();

    // state.set(GameState::Loading);

    // info!("{s:?}");
}

fn update_loading(
    // mut commands: Commands,
    asset_server: Res<AssetServer>,
    scene: Res<MyScene>,
    mut next_state: ResMut<NextState<GameState>>
) {
    // info!("update_loading");
    let state = asset_server.load_state(scene.id());
    // info!("{state:?}");

    if asset_server.is_loaded(scene.id()) {
        info!("Loaded scene");


        next_state.set(GameState::InGame);
    }
    // let x = asset_server.load("gltf/circle.gltf");
    // commands.insert_resource(MyScene(x));
}

/// Spawns a capsule mesh on the pixel-perfect layer.
fn setup_mesh(

    mut commands: Commands,
    // asset_server: Res<AssetServer>,
    scene: Res<MyScene>,
    gltf_assets: Res<Assets<Gltf>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut gltf_meshes: ResMut<Assets<GltfMesh>>,
    // mut materials: ResMut<Assets<StandardMaterial>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    info!("setup_mesh");
    let Some(gltf) = gltf_assets.get(scene.id()) else {
        panic!("scene not loaded");
    };
    // let mesh = Mesh::new(
    //     bevy::render::mesh::PrimitiveTopology::TriangleList,
    //      RenderAssetUsages::union(RenderAssetUsages::MAIN_WORLD, RenderAssetUsages::RENDER_WORLD)
    //     );
    let m = gltf_meshes.get(gltf.meshes[0].id()).unwrap().primitives[0].mesh.clone_weak();
    let mesh_data = meshes.get(m.id()).unwrap().clone().with_removed_attribute(Mesh::ATTRIBUTE_NORMAL);
    

    // let VertexAttributeValues::Float32x3(positions) = mesh_data.attribute(Mesh::ATTRIBUTE_POSITION).unwrap() else {
    //     panic!("error: not vec3");
    // };

    
    // mesh_data.indices().(|idx| {

    // });

    // for tri1 in mesh_data.triangles().unwrap() {
    //     vert_idx_to_member_tri_idx.insert(tri1., v)
    // }



    // let mesh_data = mesh_data.with_computed_flat_normals();

    commands.insert_resource(
        SimData::from_mesh(&mesh_data)
    );
    // GltfMesh

    // GltfAssetLabel::Mesh(0).
    // meshes.get(m.id());
    
    // gltf.meshes[0].id();
    let mat = StandardMaterial::from_color(Color::WHITE);
    // let m = meshes.add(m);
    // let m = meshes.get(m.id()).unwrap().clone();
    

    commands.spawn((
        // SceneRoot(gltf.default_scene.clone().unwrap()),
        Mesh2d(m),
        // Mesh3d(m),
        // MeshMaterial3d(materials.add(mat))
        MeshMaterial2d(materials.add(ColorMaterial::from_color(Color::WHITE))),
        Transform::from_scale(Vec3::splat(10.)),
        HIGH_RES_LAYERS,
    ));

    // commands.spawn((
    //     // DirectionalLight {color: Color::WHITE, shadows_enabled: false, illuminance: 100., ..default()},
    //     PointLight {
    //         // color: Color::WHITE,
    //         shadows_enabled: true,
    //         intensity: 3000.,
    //         ..default()
    //     },
    //     Transform::from_translation(2. * Vec3::Z),
    // ));
    // commands.spawn((
    //     // SceneRoot(gltf.default_scene.clone().unwrap()),
    //     // Mesh3d()
    //     // AmbientLight {color: Color::WHITE, brightness: 100.},
    //     Light
    //     // MeshMaterial3d(materials.add(mat))
    // ));
    // let x = asset_server.load("gltf/circle.gltf");

    // commands.spawn((
    //     SceneRoot(x),

    // ));
    // commands.insert_resource(MyScene(x));
    
    // let x = asset_server.load(GltfAssetLabel::Mesh(0).from_asset("gltf/circle.gltf"));
    // commands.spawn((
    //     Mesh2d(x),
    //     MeshMaterial2d(materials.add(Color::BLACK)),
    //     Transform::from_xyz(0., 2.,0. ).with_scale(Vec3::splat(32.)),
    //     PIXEL_PERFECT_LAYERS,
    // ));
}


// fn update_mesh_load(server: Res<AssetServer>, meshes: Res<Assets<Mesh>>,  mut er: EventReader<AssetEvent<Mesh>>) {
//     for e in er.read() {
//         match e {
//             // AssetEvent::Added { id } => todo!(),
//             // AssetEvent::Modified { id } => todo!(),
//             // AssetEvent::Removed { id } => todo!(),
//             // AssetEvent::Unused { id } => todo!(),
//             AssetEvent::LoadedWithDependencies { id } => {
//                 // let h= meshes.get(id);

//             },
//             _ => (),
//         }
//     }
// }




fn setup_camera(mut commands: Commands, mut images: ResMut<Assets<Image>>) {
    let canvas_size = Extent3d {
        width: RES_WIDTH,
        height: RES_HEIGHT,
        ..default()
    };

    // this Image serves as a canvas representing the low-resolution game screen
    let mut canvas = Image {
        texture_descriptor: TextureDescriptor {
            label: None,
            size: canvas_size,
            dimension: TextureDimension::D2,
            format: TextureFormat::Bgra8UnormSrgb,
            mip_level_count: 1,
            sample_count: 1,
            usage: TextureUsages::TEXTURE_BINDING
                | TextureUsages::COPY_DST
                | TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        },
        ..default()
    };

    // fill image.data with zeroes
    canvas.resize(canvas_size);

    let image_handle = images.add(canvas);

    // this camera renders whatever is on `PIXEL_PERFECT_LAYERS` to the canvas
    commands.spawn((
        Camera2d,
        Camera {
            clear_color: ClearColorConfig::Custom(Color::BLACK),
            // render before the "main pass" camera
            order: -1,
            target: RenderTarget::Image(image_handle.clone()),
            ..default()
        },
        Msaa::Off,
        InGameCamera,
        PIXEL_PERFECT_LAYERS,
    ));

    // spawn the canvas
    commands.spawn((Sprite::from_image(image_handle), Canvas, HIGH_RES_LAYERS, Transform::from_translation(-1. * Vec3::Z)));

    // the "outer" camera renders whatever is on `HIGH_RES_LAYERS` to the screen.
    // here, the canvas and one of the sample sprites will be rendered by this camera
    // commands.spawn((Camera2d, Msaa::Off, OuterCamera, HIGH_RES_LAYERS));
    commands.spawn((Camera2d::default(),
        // OrthographicProjection::default_3d(),
        // Transform::from_translation(Vec3::new(0., 0., 10.)).looking_at(Vec3::ZERO, Vec3::Y),
        Msaa::Off,
        OuterCamera,
        HIGH_RES_LAYERS,
    ));

}

/// Rotates entities to demonstrate grid snapping.
// fn rotate(time: Res<Time>, mut transforms: Query<&mut Transform, With<Rotate>>) {
//     for mut transform in &mut transforms {
//         let dt = time.delta_secs();
//         transform.rotate_z(dt);
//     }
// }

fn update_ui(timer: ResMut<MyTimer>, mut query: Query<&mut Text, With<FPSNode>>, time: Res<Time<Real>>) {
    let mut s= query.single_mut();
    // let t = t.into_inner();

    let timer = timer.into_inner();
    let delta = time.into_inner().delta();
    timer.tick(delta);

    if timer.finished() {
        let fps = 1. / delta.as_secs_f32();

        s.clear();
        s.push_str(format!("FPS {fps:.2}").as_str());
    }


    // if t.el

}

// fn setup_sim(mut commands: Commands) {
// }


fn update_soft(simdata: ResMut<SimData>, mut mesh_query: Query<&mut Mesh2d>, mut meshes: ResMut<Assets<Mesh>>) {
    // return;
    let mesh2d = mesh_query.single_mut();
    let simdata = simdata.into_inner();

    simdata.update();

    let new_mesh = &simdata.template_mesh;

    let mesh = meshes.get_mut(mesh2d.id()).unwrap();
    
    let VertexAttributeValues::Float32x3(new_positions) = new_mesh.attribute(Mesh::ATTRIBUTE_POSITION).unwrap() else {
        panic!();
    };
    
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, new_positions.clone());
    mesh.insert_indices(new_mesh.indices().unwrap().clone());
    mesh.compute_normals();
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
