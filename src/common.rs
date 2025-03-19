use bevy::asset::Handle;
use bevy::gltf::Gltf;
use bevy::prelude::{Component, Deref, DerefMut, Resource};


#[derive(Component, Deref, DerefMut)]
pub struct MeshIndices(pub Vec<u16>);


#[derive(Component, Deref, DerefMut)]
pub struct VertexIndex(pub u16);

#[derive(Resource, Deref)]
pub struct GameScene(pub Handle<Gltf>);

#[derive(Component)]
pub struct SoftBodyPoint;
