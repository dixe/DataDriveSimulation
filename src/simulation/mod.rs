use std::collections::HashMap;
use nalgebra as na;
use nalgebra::vector;


pub type EntityId = usize;
pub type V3 = na::Vector3::<f32>;

// All into regarding the simulation
#[derive(Debug)]
pub struct State {
    next_id: EntityId,
    pub spheres: ActiveSpheres,
    pub walls: Walls,
}


impl State {
    pub fn new() -> Self {
        Self {
            next_id: 1,
            spheres: ActiveSpheres::new(),
            walls: Walls::new()
        }
    }


    pub fn add_ball(&mut self, pos: V3, vel: V3, r: f32, mass: f32) -> EntityId {
        let id = self.next_id;
        self.next_id += 1;
        self.spheres.add_entity(NewBall {
            id,
            pos,
            vel,
            radius: r,
            mass
        });

        id
    }

    pub fn add_wall(&mut self, pos: V3, size: V3) {
        self.walls.add_wall(pos, size);
    }
}


#[derive(Default, Clone, Debug)]
struct Manifold {
    vel_change : V3,
    pos_correction: V3,
}



fn impulse_manifolds(state: &State) -> Vec::<Manifold> {

    // parially from
    //https://gamedevelopment.tutsplus.com/tutorials/how-to-create-a-custom-2d-physics-engine-the-basics-and-impulse-resolution--gamedev-6331

    let count = state.spheres.count();

    let mut res : Vec::<Manifold>= vec![Default::default(); count];

    let pos = &state.spheres.positions;
    let vel = &state.spheres.velocities;
    let radius = &state.spheres.radius;
    let mass = &state.spheres.mass;

    for i in 0..count {
        for j in (i + 1)..count {
            let dist = (pos[i]- pos[j]).norm();

            if radius[i] + radius[j] >= dist {

                //println!("sum_r, d = {:?}", (radius[i] + radius[j], dist));
                //println!("p_i, p_j = {:?}", (pos[i], pos[j]));

                let relative_vel = vel[j] - vel[i];
                let col_norm : V3 = (pos[j] - pos[i]).normalize();
                let pen_depth = dist - radius[i] + radius[j];

                let vel_along_norm = relative_vel.dot(&col_norm);

                if vel_along_norm > 0.0 {
                    continue;
                }

                // can also be so each ball has its own, then use lowest
                let resitution = 1.0; // elasicity e = rel_vel_after_col / rel_vel_before, so 1 is all energy preserved,

                let mut impulse_scalar = -(1.0 + resitution) * vel_along_norm;
                impulse_scalar /= 1.0/mass[i] + 1.0/mass[j];

                let impulse : V3 = col_norm * impulse_scalar;


                res[i].vel_change -= 1.0/mass[i] * impulse;
                res[j].vel_change += 1.0/mass[j] * impulse;

                let percent = 0.1; // between 0.2 and 0.8 usually
                let correction : V3 = pen_depth / (1.0/mass[i] + 1.0/mass[j]) * percent * col_norm;
                res[i].pos_correction -= 1.0/mass[i] * correction;
                res[j].pos_correction += 1.0/mass[j] * correction;

                //println!("{:?}", impulse);
                //println!("res ={:.2?}", res);
            }
        }
    }

    res
}



fn impulse_walls(state: &mut State, manifolds: &mut Vec::<Manifold>) {

    let count = state.spheres.count();

    let pos = &state.spheres.positions;
    let vel = &state.spheres.velocities;
    let radius = &state.spheres.radius;
    let mass = &state.spheres.mass;


    let wall_p = &state.walls.positions;
    let sizes = &state.walls.sizes;

    let wall_count = state.walls.count();

    for i in 0..count {
        for w_i in 0..wall_count {
            let n = wall_p[w_i] - pos[i];
            let extent = sizes[w_i]/ 2.0;

            let closest =  na::clamp(n, -extent, extent);

            let closest = vector![
                na::clamp(n.x, -extent.x, extent.x),
                na::clamp(n.y, -extent.y, extent.y),
                na::clamp(n.z, -extent.z, extent.z)];

            // TODO: maybe handle closest==n that means center of sphere is inside the cube

            let mut normal = (wall_p[w_i] - closest) - pos[i];
            let dist = normal.norm();
            normal = normal.normalize();


            if dist <= radius[i] {
                // collision


                let vel_along_norm = vel[i].dot(&normal);

                let resitution = 1.0;

                let mut impulse_scalar = (1.0 + resitution) * vel_along_norm;
                impulse_scalar /= 1.0/mass[i];

                let impulse : V3 = normal * impulse_scalar;
                manifolds[i].vel_change -= 1.0/mass[i] * impulse;


                let percent = 0.1; // between 0.2 and 0.8 usually

                let pen_depth = radius[i] - dist;
                let correction : V3 = (pen_depth / 1.0/mass[i]) * percent * normal;

                manifolds[i].pos_correction -= 1.0/mass[i] * correction;
            }
        }
    }
}


pub fn step(state: &mut State, dt: f32) {

    let count = state.spheres.count();
    // get acceleration of each ball, calculated from collision
    let mut manifolds = impulse_manifolds(state);

    impulse_walls(state, &mut manifolds);

    //
    let pos = &mut state.spheres.positions;
    let vel = &mut state.spheres.velocities;


    for i in 0..count {
        vel[i] += manifolds[i].vel_change;
        pos[i] += vel[i] * dt + manifolds[i].pos_correction;
    }
}




#[derive(Debug)]
pub struct ActiveSpheres {
    id_to_index : HashMap::<EntityId,usize>,
    pub positions: Vec::<V3>,
    pub velocities: Vec::<V3>,
    pub radius: Vec::<f32>,
    pub mass: Vec::<f32>,
}

impl ActiveSpheres {

    pub fn new() -> Self {
        Self {
            id_to_index : HashMap::new(),
            positions: vec![],
            velocities: vec![],
            radius: vec![],
            mass: vec![]
        }
    }

    pub fn count(&self) -> usize {
        self.positions.len()
    }

    pub fn add_entity(&mut self, new: NewBall) -> usize {

        let index = self.positions.len();

        self.id_to_index.insert(new.id, index);
        self.positions.push(new.pos);
        self.velocities.push(new.vel);
        self.radius.push(new.radius);
        self.mass.push(new.mass);
        index
    }
}

#[derive(Debug)]
pub struct NewBall {
    pub id: EntityId,
    pub pos: V3,
    pub vel: V3,
    pub radius: f32,
    pub mass : f32
}


#[derive(Debug)]
pub struct Walls {
    pub positions: Vec::<V3>, // center position of wall
    pub sizes: Vec::<V3>, // size is width, depth and height, with
        // rotation
}

impl Walls {
    pub fn new() -> Self {
        Self {
            positions: vec![],
            sizes: vec![]
        }
    }

    pub fn count(&self) -> usize {
        self.positions.len()
    }

    pub fn add_wall(&mut self, center: V3, size: V3) {
        self.positions.push(center);
        self.sizes.push(size);

    }
}
