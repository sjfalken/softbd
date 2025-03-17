use avian2d::collision::{Collider, CollisionLayers};
use avian2d::prelude::{DistanceJoint, Joint, Mass, PhysicsLayer, RigidBody, SweptCcd};
use bevy::asset::Assets;
use bevy::color::Color;
use bevy::gltf::{Gltf, GltfMesh};
use bevy::math::{NormedVectorSpace, Vec3};
use bevy::prelude::*;
use bevy::render::mesh::{Indices, VertexAttributeValues};

use crate::common::*;

// use crate::{GameLayer, MeshIndices, MyScene, SimVertex,  VertexIndex};


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


#[derive(PhysicsLayer, Default)]
pub enum GameLayer {
    #[default]
    Default,
    SoftBodyPointLayer,
}

#[derive(Component)]
#[require(Mesh2d)]
pub struct SoftBody;


#[derive(Component)]
pub struct SoftBodyPoint;
#[derive(Bundle)]
pub struct SoftBodyPointBundle {
    mesh_indices: MeshIndices,
    vertex_index: VertexIndex,
    transform: Transform,
    mass: Mass,
    softbody_point: SoftBodyPoint,
    collider: Collider,
    ccd: SweptCcd,
    // spec_ccd: SpeculativeMargin,
    collision_layers: CollisionLayers,
    body: RigidBody,
    // joint: DistanceJoint,
}

pub fn setup_softbody(
    mut commands: Commands,
    scene: Res<GameScene>,
    gltf_assets: Res<Assets<Gltf>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    gltf_meshes: ResMut<Assets<GltfMesh>>,
    // mut solver_config: ResMut<SolverConfig>,
) {
    // solver_config.restitution_iterations = 3;

    let Some(gltf) = gltf_assets.get(scene.id()) else {
        panic!("scene not loaded");
    };
    let mh = gltf_meshes.get(gltf.meshes[0].id()).unwrap().primitives[0].mesh.clone_weak();
    let mut m = meshes.get(mh.id()).unwrap().clone().with_removed_attribute(Mesh::ATTRIBUTE_NORMAL);

    let VertexAttributeValues::Float32x3(positions) = m.attribute(Mesh::ATTRIBUTE_POSITION).unwrap() else {
        panic!("error: not vec3");
    };

    let mut vertices = positions.clone().iter().enumerate().map(|(i, v)| {
        (SimVertex(Vec3::new(v[0], v[1], v[2])), vec![i])
    }).collect::<Vec<(SimVertex, Vec<usize>)>>();

    let mut indices: Vec<u16> = Vec::new();
    indices.resize(vertices.len(), 0);

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
    // template_mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, VertexAttributeValues::Float32x3(positions));
    // template_mesh.insert_indices(Indices::U16(indices));

    // let mut tri_membership: Vec<Vec<usize>> = Vec::new();
    let points = points_with_orig_idx.iter().enumerate().map(|(i, (p, orig))| {
        for &x in orig.iter() {
            indices[x] = i as u16;
        }
        SoftBodyPointBundle {
            mass: Mass::from(1.0),
            transform: Transform::from_translation(p.0),
            mesh_indices: MeshIndices(orig.iter().map(|x| {*x as u16}).collect()),
            softbody_point: SoftBodyPoint,
            body: RigidBody::Dynamic,
            collider: Collider::rectangle(0.1, 0.1), // TODO fine tune
            ccd: SweptCcd::default(),
            // spec_ccd: SpeculativeMargin(100.0),
            collision_layers: CollisionLayers::new(GameLayer::SoftBodyPointLayer, GameLayer::Default),
            vertex_index: VertexIndex(i as u16),
        }
    }).collect::<Vec<SoftBodyPointBundle>>();

    m.remove_attribute(Mesh::ATTRIBUTE_POSITION);
    m.insert_attribute(Mesh::ATTRIBUTE_POSITION, points_with_orig_idx.iter().map(|(a, b)| {a.0.to_array()}).collect::<Vec<[f32;3]>>());
    m.insert_indices(Indices::U16(indices));
    let point_entities = points.into_iter().map(|p|{
        commands.spawn(p).id()
    }).collect::<Vec<Entity>>();

    let mut b = commands.spawn((
        Mesh2d(meshes.add(m)),
        SoftBody,
        MeshMaterial2d(materials.add(ColorMaterial::from_color(Color::WHITE))),
        Transform::from_scale(Vec3::splat(10.)).with_translation(Vec3::new(0., 5., 0.)),
    ));

    b.add_children(point_entities.as_slice());
}


pub fn setup_joints(
    mut commands: Commands,
    point_entities: Query<Entity, With<SoftBodyPoint>>,
    point_mesh_indices: Query<&MeshIndices, With<SoftBodyPoint>>,
    point_transforms: Query<&GlobalTransform, With<SoftBodyPoint>>,

) {
    // let mut ec = commands.entity(softbody_entity.into_inner());
    //
    // ec.insert((
    //     // MeshMaterial2d(materials.add(ColorMaterial::from_color(Color::WHITE))),
    //     // Transform::from_scale(Vec3::splat(10.)).with_translation(Vec3::new(0., 5., 0.)),
    //     // HIGH_RES_LAYERS,
    // ));

    let mut joints: Vec<DistanceJoint> = Vec::new();

    let point_entities = point_entities.iter().collect::<Vec<_>>();
    (0..point_entities.len()).for_each(|i| {

        ((i+1)..point_entities.len()).for_each(|j| {
            // if i == j {
            //     return;
            // }
            let t1: Vec<usize> = point_mesh_indices.get(point_entities[i]).unwrap().iter().map(|x| *x as usize / 3).collect();
            let t2: Vec<usize> = point_mesh_indices.get(point_entities[j]).unwrap().iter().map(|x| *x as usize / 3).collect();

            for t in t1 {
                if t2.contains(&t) {

                    let p1 = point_transforms.get(point_entities[i]).unwrap().translation();
                    let p2 = point_transforms.get(point_entities[j]).unwrap().translation();
                    // let p2 = point_transforms.get(point_entities[j]).unwrap().
                    let rest = (p2 - p1).norm();
                    // info!("{p1:?} {p2:?}");
                    joints.push(
                        DistanceJoint::new(point_entities[i], point_entities[j])
                            .with_rest_length(rest)
                            .with_angular_velocity_damping(1000.)
                            // .with_limits()
                            // .with_limits(rest *0.9, rest*1.1)
                            .with_compliance(0.0001)
                    );

                    // let last = joints[joints.len() - 1];
                }
            }
        });
    });


    commands.spawn_batch(joints);

}
