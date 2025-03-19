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
    // mut player: Query<&mut PlayerState>
    time: Res<Time>,
) {
    let delta_secs = time.delta_secs();


    // info!("{:?}", avg_vel.norm());

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

        // let area_min = 0.8 * self.signed_rest_area;
        // let area_max = 1.2 * self.signed_rest_area;
        let area_min = self.signed_rest_area;
        let area_max = self.signed_rest_area;
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
        // let delta_lagrange =
        //     self.compute_lagrange_update_with_gradients(self.lagrange, area_difference, &g, &w, self.compliance, dt);

        let delta_lagrange =
            self.compute_lagrange_update(self.lagrange, area_difference, &w, self.compliance, dt);

        self.lagrange += delta_lagrange;
        
        if delta_lagrange.abs() > Scalar::EPSILON {
            body1.accumulated_translation.0 += delta_lagrange * w1 * dir1;
            body2.accumulated_translation.0 += delta_lagrange * w2 * dir2;
            body3.accumulated_translation.0 += delta_lagrange * w3 * dir3;

            // TODO I don't think this is contributing anything because of the zero vectors
            let delta_angle1 = Self::get_delta_rot(
                body1.effective_global_angular_inertia().inverse(), Vector::ZERO, delta_lagrange * dir1);
            let delta_angle2 = Self::get_delta_rot(
                body2.effective_global_angular_inertia().inverse(), Vector::ZERO, delta_lagrange * dir2);
            let delta_angle3 = Self::get_delta_rot(
                body3.effective_global_angular_inertia().inverse(), Vector::ZERO, delta_lagrange * dir3);
            *body1.rotation = body1.rotation.add_angle(delta_angle1);
            *body2.rotation = body2.rotation.add_angle(delta_angle2);
            *body3.rotation = body3.rotation.add_angle(delta_angle3);
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






