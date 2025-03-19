use avian2d::collision::{Collider, CollisionLayers};
use avian2d::dynamics::solver::xpbd::{AngularConstraint, PositionConstraint, XpbdConstraint};
use avian2d::math::{Scalar, Vector};
use avian2d::prelude::*;
use bevy::asset::Assets;
use bevy::color::Color;
use bevy::ecs::entity::MapEntities;
use bevy::gltf::{Gltf, GltfMesh};
use bevy::math::{NormedVectorSpace, Vec3};
use bevy::prelude::*;
use bevy::render::mesh::{Indices, VertexAttributeValues};

use crate::common::*;
use crate::constr::TriAreaConstraint;


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
pub struct SoftBody {
    vel: Vector
}

#[derive(Component, Deref, DerefMut)]
pub struct TriangleList(Vec<(Entity, Entity, Entity)>);



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

    let points = points_with_orig_idx.iter().enumerate().map(|(i, (p, orig))| {
        for &x in orig.iter() {
            indices[x] = i as u16;
        }
        (
            Transform::from_translation(p.0),
            MeshIndices(orig.iter().map(|x| {*x as u16}).collect()),
            // MeshTriangleNumber()
            SoftBodyPoint,
            RigidBody::Dynamic,
            ExternalImpulse::new(Vector::ZERO),
            Collider::circle(0.1), // TODO fine tune
            // ColliderDensity(1.0),
            SweptCcd::default(),
            // spec_ccd: SpeculativeMargin(100.0),
            CollisionLayers::new(GameLayer::SoftBodyPointLayer, GameLayer::Default),
            LockedAxes::ROTATION_LOCKED,
            VertexIndex(i as u16),
        )
    }).collect::<Vec<_>>();
    let point_entities = points.into_iter().map(|p|{
        commands.spawn(p).id()
    }).collect::<Vec<Entity>>();

    let mut triangles = Vec::new();
    
    for t in 0..(indices.len() / 3) {
        triangles.push((
                point_entities[indices[t*3] as usize], 
                point_entities[indices[t*3+1] as usize], 
                point_entities[indices[t*3+2] as usize]
            ));
    }
    
    m.remove_attribute(Mesh::ATTRIBUTE_POSITION);
    m.insert_attribute(Mesh::ATTRIBUTE_POSITION, points_with_orig_idx.iter().map(|(a, _)| {a.0.to_array()}).collect::<Vec<[f32;3]>>());
    m.insert_indices(Indices::U16(indices));
    
    let mut b = commands.spawn((
        // Mass::from(100.),
        Mesh2d(meshes.add(m)),
        SoftBody {vel: Vector::ZERO},
        // RigidBody::Dynamic,
        TriangleList(triangles),
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
    softbody_triangles: Single<&TriangleList, With<SoftBody>>,
    // meshes: Res<Assets<Mesh>>,
) {
    let mut joints: Vec<TriAreaConstraint> = Vec::new();
    
    let vto2d = |v: Vec3| {
        Vec2::new(v.x, v.y)
    };
    
    let triangles = softbody_triangles.iter().collect::<Vec<_>>();
    for i in 0..triangles.len() {
        let (e1, e2, e3) = triangles[i];
        
        let p1 = point_transforms.get(*e1).unwrap().translation();
        let p2 = point_transforms.get(*e2).unwrap().translation();
        let p3 = point_transforms.get(*e3).unwrap().translation();
        
        let p1 = vto2d(p1);
        let p2 = vto2d(p2);
        let p3 = vto2d(p3);
        
        let area = TriAreaConstraint::compute_signed_area(p1, p2, p3);
        
        joints.push(TriAreaConstraint::new([*e1, *e2, *e3], area, 0.05));
    }
    
    commands.spawn_batch(joints);
    
    let mut joints: Vec<DistanceJoint> = Vec::new();
    
    let point_entities = point_entities.iter().collect::<Vec<_>>();
    
    for i in 0..point_entities.len() {
        for j in (i+1)..point_entities.iter().len() {

            let t1: Vec<usize> = point_mesh_indices.get(point_entities[i]).unwrap().iter().map(|x| *x as usize / 3).collect();
            let t2: Vec<usize> = point_mesh_indices.get(point_entities[j]).unwrap().iter().map(|x| *x as usize / 3).collect();

            for t in t1 {
                if t2.contains(&t) {

                    let p1 = point_transforms.get(point_entities[i]).unwrap().translation();
                    let p2 = point_transforms.get(point_entities[j]).unwrap().translation();
                    // let p2 = point_transforms.get(point_entities[j]).unwrap().
                    let rest = (p2 - p1).norm();
                    joints.push(
                        DistanceJoint::new(point_entities[i], point_entities[j])
                            .with_rest_length(rest)
                            .with_compliance(0.00001)
                    );

                    // let last = joints[joints.len() - 1];
                }
            }
        }
    }

    commands.spawn_batch(joints);
}

pub fn update_softbody(
    // points: Query<&LinearVelocity, With<SoftBodyPoint>>,
    body: Single<&mut SoftBody>,
    mut tri_constr: Query<&mut TriAreaConstraint>,
) {

    // let mut count = 0;
    // let avg_vel = points.iter().map(|v| v.0).reduce(|a, b| {
    //     count += 1;
    //     a + b
    // }).unwrap() / count as f32;
    // 
    // body.into_inner().vel = avg_vel;
    
    
    // let a0 = 0.1;
    // let ascale = 0.05;
    // 
    // tri_constr.iter_mut().for_each(|mut c| {
    //     c.compliance = a0 / (1. + ascale * avg_vel.norm());
    // });
    // 
    // let a0 = 0.000001;
    // let ascale = 0.1;
    // 
    // dist_constr.iter_mut().for_each(|mut c| {
    //     c.compliance = a0 / (1. + ascale * avg_vel.norm());
    // });
    
}
