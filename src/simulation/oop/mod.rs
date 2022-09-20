use std::collections::HashMap;
use nalgebra as na;
use nalgebra::vector;


pub type EntityId = usize;
pub type V3 = na::Vector3::<f32>;

// All into regarding the simulation
#[derive(Debug)]
pub struct State {
    next_id: EntityId,
    pub spheres: Vec::<Sphere>,
    pub walls: Walls,
}


impl State {
    pub fn new() -> Self {
        Self {
            next_id: 1,
            spheres: Vec::new(),
            walls: Walls::new()
        }
    }


    pub fn add_ball(&mut self, pos: V3, vel: V3, r: f32, mass: f32) -> EntityId {
        let id = self.next_id;
        self.next_id += 1;
        self.spheres.push( Sphere {
            id,
            pos,
            vel,
            r,
            mass,
        });

        id
    }

    pub fn add_wall(&mut self, pos: V3, size: V3) {
        self.walls.add_wall(pos, size);
    }


    pub fn sphere_radius(&self) -> Vec::<f32> {
        self.spheres.iter().map(|s| s.r).collect()
    }

    pub fn sphere_positions(&self) -> Vec::<V3> {
        self.spheres.iter().map(|s| s.pos).collect()
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

    let count = state.spheres.len();

    let mut res : Vec::<Manifold>= vec![Default::default(); count];


    for i in 0..count {
        for j in (i + 1)..count {

            let si = &state.spheres[i];
            let sj = &state.spheres[j];


            let dist = (si.pos- sj.pos).norm();

            if si.r + sj.r >= dist {

                let relative_vel = sj.vel - si.vel;
                let col_norm : V3 = (sj.pos - si.pos).normalize();
                let pen_depth = dist - si.r + sj.r;

                let vel_along_norm = relative_vel.dot(&col_norm);

                if vel_along_norm > 0.0 {
                    continue;
                }

                // can also be so each ball has its own, then use lowest
                let resitution = 1.0; // elasicity e = rel_vel_after_col / rel_vel_before, so 1 is all energy preserved,

                let mut impulse_scalar = -(1.0 + resitution) * vel_along_norm;
                impulse_scalar /= 1.0/si.mass + 1.0/sj.mass;

                let impulse : V3 = col_norm * impulse_scalar;


                res[i].vel_change -= 1.0/si.mass * impulse;
                res[j].vel_change += 1.0/sj.mass * impulse;

                let percent = 0.1; // between 0.2 and 0.8 usually
                let correction : V3 = pen_depth / (1.0/si.mass + 1.0/sj.mass) * percent * col_norm;
                res[i].pos_correction -= 1.0/si.mass * correction;
                res[j].pos_correction += 1.0/sj.mass * correction;

                //println!("{:?}", impulse);
                //println!("res ={:.2?}", res);
            }
        }
    }

    res
}



fn impulse_walls(state: &mut State, manifolds: &mut Vec::<Manifold>) {

    let count = state.spheres.len();

    let wall_p = &state.walls.positions;
    let sizes = &state.walls.sizes;

    let wall_count = state.walls.count();

    for i in 0..count {
        for w_i in 0..wall_count {

            let si = &state.spheres[i];

            let n = wall_p[w_i] - si.pos;
            let extent = sizes[w_i]/ 2.0;

            let closest =  na::clamp(n, -extent, extent);

            let closest = vector![
                na::clamp(n.x, -extent.x, extent.x),
                na::clamp(n.y, -extent.y, extent.y),
                na::clamp(n.z, -extent.z, extent.z)];

            // TODO: maybe handle closest==n that means center of sphere is inside the cube

            let mut normal = (wall_p[w_i] - closest) - si.pos;
            let dist = normal.norm();
            normal = normal.normalize();


            if dist <= si.r {
                // collision


                let vel_along_norm = si.vel.dot(&normal);

                let resitution = 1.0;

                let mut impulse_scalar = (1.0 + resitution) * vel_along_norm;
                impulse_scalar /= 1.0/si.mass;

                let impulse : V3 = normal * impulse_scalar;
                manifolds[i].vel_change -= 1.0/si.mass * impulse;


                let percent = 0.1; // between 0.2 and 0.8 usually

                let pen_depth = si.r - dist;
                let correction : V3 = (pen_depth / 1.0/si.mass) * percent * normal;

                manifolds[i].pos_correction -= 1.0/si.mass * correction;
            }
        }
    }
}


pub fn step(state: &mut State, dt: f32) {

    let count = state.spheres.len();
    // get acceleration of each ball, calculated from collision
    let mut manifolds = impulse_manifolds(state);

    impulse_walls(state, &mut manifolds);

    for i in 0..count {
        let s = &mut state.spheres[i];
        s.vel += manifolds[i].vel_change;
        s.pos += s.vel * dt + manifolds[i].pos_correction;
    }
}



#[derive(Debug)]
pub struct Sphere {
    pub id: usize,
    pub pos: V3,
    pub vel: V3,
    pub r: f32,
    pub mass: f32,
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
