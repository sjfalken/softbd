use avian2d::dynamics::solver::xpbd::{AngularConstraint, PositionConstraint, XpbdConstraint};
use avian2d::math::{Matrix3, Quaternion, Scalar, Vector};
use avian2d::prelude::*;
use bevy::{
    ecs::{
        entity::{EntityMapper, MapEntities},
        reflect::ReflectMapEntities,
    },
    prelude::*,
};
use bevy::math::NormedVectorSpace;
use bevy::reflect::Tuple;
use bevy::render::render_graph::Edge;

pub fn iterative_constraint_begin<C: XpbdConstraint<ENTITY_COUNT> + Component, const ENTITY_COUNT: usize>(
    // mut commands: Commands,
    // mut bodies: Query<RigidBodyQuery, Without<RigidBodyDisabled>>,
    mut constraints: Query<&mut C, (Without<RigidBody>, Without<JointDisabled>)>,
    // time: Res<Time>,
) {
    // let delta_secs = time.delta_secs();
    // Clear Lagrange multipliers
    constraints
        .iter_mut()
        .for_each(|mut c| c.clear_lagrange_multipliers());
}

pub fn iterative_constraint_solve<C: XpbdConstraint<ENTITY_COUNT> + Component, const ENTITY_COUNT: usize>(
    mut commands: Commands,
    mut bodies: Query<RigidBodyQuery, Without<RigidBodyDisabled>>,
    mut constraints: Query<&mut C, (Without<RigidBody>, Without<JointDisabled>)>,
    time: Res<Time>,
) {
    let delta_secs = time.delta_secs();
    
    for constraint in &mut constraints {
        // Get components for entities
        if let Ok(bodies) = bodies.get_many_mut(constraint.entities()) {
            iterative_constraint_solve_loop(&mut commands, bodies, constraint, delta_secs);
        }
    }
}
fn iterative_constraint_solve_loop<C: XpbdConstraint<ENTITY_COUNT> + Component, const ENTITY_COUNT: usize>(
    commands: &mut Commands,
    mut bodies: [RigidBodyQueryItem; ENTITY_COUNT],
    mut constraint: Mut<C>,
    delta_time: Scalar,
) {

    let none_dynamic = bodies.iter().all(|body| !body.rb.is_dynamic());
    let all_inactive = bodies
        .iter()
        .all(|body| body.rb.is_static() || body.is_sleeping);

    if none_dynamic || all_inactive {
        return;
    }

    // At least one of the participating bodies is active, so wake up any sleeping bodies
    for body in &mut bodies {
        // Reset the sleep timer
        if let Some(time_sleeping) = body.time_sleeping.as_mut() {
            time_sleeping.0 = 0.0;
        }

        if body.is_sleeping {
            commands.queue(WakeUpBody(body.entity));
        }
    }

    // Get the bodies as an array and solve the constraint
    if let Ok(bodies) = bodies
        .iter_mut()
        .collect::<Vec<&mut RigidBodyQueryItem>>()
        .try_into()
    {
        constraint.solve(bodies, delta_time);
    }
}



#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
// #[reflect(Debug, Component, MapEntities, PartialEq)]
// #[require(RigidBody)]
pub struct TriAreaConstraint {
    
    /// First entity constrained by the joint.
    pub entity1: Entity,
    /// Second entity constrained by the joint.
    pub entity2: Entity,
    
    pub entity3: Entity,
    
    pub signed_rest_area: Scalar,
    /// The extents of the allowed relative translation between the attached bodies.
    // pub length_limits: Option<DistanceLimit>,
    /// Linear damping applied by the joint.
    // pub damping_linear: Scalar,
    /// Angular damping applied by the joint.
    // pub damping_angular: Scalar,
    /// Lagrange multiplier for the positional correction.
    pub lagrange: Scalar,
    /// The joint's compliance, the inverse of stiffness, has the unit meters / Newton.
    pub compliance: Scalar,
    pub force: Vector,
}

impl XpbdConstraint<3> for TriAreaConstraint {
    fn entities(&self) -> [Entity; 3] {
        [self.entity1, self.entity2, self.entity3]
    }

    fn solve(&mut self, bodies: [&mut RigidBodyQueryItem; 3], dt: Scalar) {
        self.constrain_area(bodies, dt);
    }

    fn clear_lagrange_multipliers(&mut self) {
        self.lagrange = 0.0;
    }
}

impl TriAreaConstraint {

    pub fn new(entities: [Entity; 3], rest: f32, compliance: Scalar) -> Self {
        Self {
            entity1: entities[0],
            entity2: entities[1],
            entity3: entities[2],
            signed_rest_area: rest,
            force: Vector::ZERO,
            compliance,
            lagrange: 0.,
        }
    }
    fn constrain_area(&mut self, bodies: [&mut RigidBodyQueryItem; 3], dt: Scalar) {
        let [body1, body2, body3] = bodies;
        // let world_r1 = *body1.rotation * self.local_anchor1;
        // let world_r2 = *body2.rotation * self.local_anchor2;
        // 
        // // If min and max limits aren't specified, use rest length
        // // TODO: Remove rest length, just use min/max limits.
        // let limits = self
        //     .length_limits
        //     .unwrap_or(DistanceLimit::new(self.rest_length, self.rest_length));
        // 
        
        let p1 = body1.current_position();
        let p2 = body2.current_position();
        let p3 = body3.current_position();
        
        let area_min = 0.9 * self.signed_rest_area;
        let area_max = 1.1 * self.signed_rest_area;
        // calculated from determinant
        let signed_area = Self::compute_signed_area(p1, p2, p3);
        let area_difference1 = signed_area - area_max;
        let area_difference2 = area_min - self.signed_rest_area;
        
        let area_difference = 
            if signed_area > area_max {
                signed_area - area_max
            } else if signed_area < area_min {
                signed_area - area_min
            } else {
                0.
            };
        
        // gradients of Area w.r.t. each vertex
        let grad_area1 = 0.5 * Vector::new(p2.y - p3.y, p3.x - p2.x);
        let grad_area2 = 0.5 * Vector::new(p3.y - p1.y, p1.x - p3.x);
        let grad_area3 = 0.5 * Vector::new(p1.y - p2.y, p2.x - p1.x);

        // Avoid division by zero and unnecessary computation
        if area_difference.abs() < Scalar::EPSILON {
            return;
        }
        
        let g = [grad_area1, grad_area2, grad_area3];
        
        let g_dir_and_len = |gi: Vector| {
            if (gi.length() < Scalar::EPSILON) {
                (Vector::ZERO, 0.0)
            } else {
                (gi.normalize(), gi.norm())
            }
        };
        
        let (dir1, len1) = g_dir_and_len(g[0]);
        let (dir2, len2) = g_dir_and_len(g[1]);
        let (dir3, len3) = g_dir_and_len(g[2]);


        let w1 = self.compute_generalized_inverse_mass(body1, p1, dir1);
        let w2 = self.compute_generalized_inverse_mass(body2, p2, dir2);
        let w3 = self.compute_generalized_inverse_mass(body3, p3, dir3);

        let w = [w1, w2, w3];
        
        // TODO maybe use this for more accuracy
        let delta_lagrange =
            self.compute_lagrange_update_with_gradients(self.lagrange, area_difference, &g, &w, self.compliance, dt);
        
        // let delta_lagrange =
        //     self.compute_lagrange_update(self.lagrange, area_difference, &w, self.compliance, dt);
        
        self.lagrange += delta_lagrange;
        
        // TODO angular corrections
        if delta_lagrange.abs() > Scalar::EPSILON {
            body1.accumulated_translation.0 += delta_lagrange * w1 * dir1;
            body2.accumulated_translation.0 += delta_lagrange * w2 * dir2;
            body3.accumulated_translation.0 += delta_lagrange * w3 * dir3;
        };
    }
    
    pub fn compute_signed_area(p1: Vector, p2: Vector, p3: Vector) -> Scalar {
        0.5 * (p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y))
    }

    fn get_delta_rot(inverse_inertia: Scalar, r: Vector, p: Vector) -> Scalar {
        // Equation 8/9 but in 2D
        inverse_inertia * r.perp_dot(p)
    }


    /// Computes the generalized inverse mass of a body when applying a positional correction
    /// at point `r` along the vector `n`.
    fn compute_generalized_inverse_mass(
        &self,
        body: &RigidBodyQueryItem,
        r: Vector,
        n: Vector,
    ) -> Scalar {
        if body.rb.is_dynamic() {
            body.mass.inverse() + body.angular_inertia.inverse() * r.perp_dot(n).powi(2)
        } else {
            // Static and kinematic bodies are a special case, where 0.0 can be thought of as infinite mass.
            0.0
        }
    }


    /// Computes the force acting along the constraint using the equation f = lambda * n / h^2
    fn compute_force(&self, lagrange: Scalar, direction: Vector, dt: Scalar) -> Vector {
        lagrange * direction / dt.powi(2)
    }
}

// impl PositionConstraint for TriAreaConstraint {}

// impl AngularConstraint for DistanceJoint {}

impl MapEntities for TriAreaConstraint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.map_entity(self.entity1);
        self.entity2 = entity_mapper.map_entity(self.entity2);
        self.entity3 = entity_mapper.map_entity(self.entity3);
    }
}

#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
pub struct EdgeDistanceConstraint {
    /// First entity constrained by the joint.
    pub entity1: Entity,
    /// Second entity constrained by the joint.
    pub entity2: Entity,
    /// Attachment point on the first body.
    pub local_anchor1: Vector,
    /// Attachment point on the second body.
    pub local_anchor2: Vector,
    /// The distance the attached bodies will be kept relative to each other.
    pub rest_length: Scalar,
    /// The extents of the allowed relative translation between the attached bodies.
    pub length_limits: Option<DistanceLimit>,
    /// Linear damping applied by the joint.
    pub damping_linear: Scalar,
    /// Angular damping applied by the joint.
    pub damping_angular: Scalar,
    /// Lagrange multiplier for the positional correction.
    pub lagrange: Scalar,
    /// The joint's compliance, the inverse of stiffness, has the unit meters / Newton.
    pub compliance: Scalar,
    /// The force exerted by the joint.
    pub force: Vector,
}



impl XpbdConstraint<2> for EdgeDistanceConstraint {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }

    fn clear_lagrange_multipliers(&mut self) {
        self.lagrange = 0.0;
    }

    fn solve(&mut self, bodies: [&mut RigidBodyQueryItem; 2], dt: Scalar) {
        self.force = self.constrain_length(bodies, dt);
    }
}

impl Joint for EdgeDistanceConstraint {
    fn new(entity1: Entity, entity2: Entity) -> Self {
        Self {
            entity1,
            entity2,
            local_anchor1: Vector::ZERO,
            local_anchor2: Vector::ZERO,
            rest_length: 0.0,
            length_limits: None,
            damping_linear: 0.0,
            damping_angular: 0.0,
            lagrange: 0.0,
            compliance: 0.0,
            force: Vector::ZERO,
        }
    }

    fn with_compliance(self, compliance: Scalar) -> Self {
        Self { compliance, ..self }
    }

    fn with_local_anchor_1(self, anchor: Vector) -> Self {
        Self {
            local_anchor1: anchor,
            ..self
        }
    }

    fn with_local_anchor_2(self, anchor: Vector) -> Self {
        Self {
            local_anchor2: anchor,
            ..self
        }
    }

    fn with_linear_velocity_damping(self, damping: Scalar) -> Self {
        Self {
            damping_linear: damping,
            ..self
        }
    }

    fn with_angular_velocity_damping(self, damping: Scalar) -> Self {
        Self {
            damping_angular: damping,
            ..self
        }
    }

    fn local_anchor_1(&self) -> Vector {
        self.local_anchor1
    }

    fn local_anchor_2(&self) -> Vector {
        self.local_anchor2
    }

    fn damping_linear(&self) -> Scalar {
        self.damping_linear
    }

    fn damping_angular(&self) -> Scalar {
        self.damping_angular
    }
}

impl EdgeDistanceConstraint {
    /// Constrains the distance the bodies with no constraint on their rotation.
    ///
    /// Returns the force exerted by this constraint.
    fn constrain_length(&mut self, bodies: [&mut RigidBodyQueryItem; 2], dt: Scalar) -> Vector {
        let [body1, body2] = bodies;
        let world_r1 = *body1.rotation * self.local_anchor1;
        let world_r2 = *body2.rotation * self.local_anchor2;

        // If min and max limits aren't specified, use rest length
        // TODO: Remove rest length, just use min/max limits.
        let limits = self
            .length_limits
            .unwrap_or(DistanceLimit::new(self.rest_length, self.rest_length));

        // Compute the direction and magnitude of the positional correction required
        // to keep the bodies within a certain distance from each other.
        let (dir, distance) = limits.compute_correction(
            body1.current_position() + world_r1,
            body2.current_position() + world_r2,
        );

        // Avoid division by zero and unnecessary computation
        if distance.abs() < Scalar::EPSILON {
            return Vector::ZERO;
        }

        // Compute generalized inverse masses (method from PositionConstraint)
        let w1 = PositionConstraint::compute_generalized_inverse_mass(self, body1, world_r1, dir);
        let w2 = PositionConstraint::compute_generalized_inverse_mass(self, body2, world_r2, dir);
        let w = [w1, w2];

        // Compute Lagrange multiplier update, essentially the signed magnitude of the correction
        let delta_lagrange =
            self.compute_lagrange_update(self.lagrange, distance, &w, self.compliance, dt);
        self.lagrange += delta_lagrange;

        // Apply positional correction (method from PositionConstraint)
        self.apply_positional_lagrange_update(
            body1,
            body2,
            delta_lagrange,
            dir,
            world_r1,
            world_r2,
        );

        // Return constraint force
        self.compute_force(self.lagrange, dir, dt)
    }

    /// Sets the minimum and maximum distances between the attached bodies.
    pub fn with_limits(self, min: Scalar, max: Scalar) -> Self {
        Self {
            length_limits: Some(DistanceLimit::new(min, max)),
            ..self
        }
    }

    /// Sets the joint's rest length, or distance the bodies will be kept at.
    pub fn with_rest_length(self, rest_length: Scalar) -> Self {
        Self {
            rest_length,
            ..self
        }
    }
}

impl PositionConstraint for EdgeDistanceConstraint {}

impl AngularConstraint for EdgeDistanceConstraint {}

impl MapEntities for EdgeDistanceConstraint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.map_entity(self.entity1);
        self.entity2 = entity_mapper.map_entity(self.entity2);
    }
}
