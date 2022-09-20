use std::collections::HashMap;
use nalgebra as na;
use nalgebra::vector;
use quadtree::{QuadTree, Rect, Point, Query};

pub type EntityId = usize;
pub type V3 = na::Vector3::<f32>;

// All into regarding the simulation
#[derive(Debug)]
pub struct State {
    next_id: EntityId,
    pub(crate) spheres: ActiveSpheres,
    pub(crate) walls: Walls,
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

    pub fn sphere_radius(&self) -> &Vec::<f32> {
        &self.spheres.radius
    }

    pub fn sphere_positions(&self) -> &Vec::<V3> {
        &self.spheres.positions
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

        let query_p = Query::point(pos[i].x as i32, pos[i].y as i32);

        //let near_p = state.spheres.positions2.query(&query_p);

        let r = Rect::from_points(Point { x: (pos[i].x - radius[i]) as i32, y: (pos[i].y - radius[i]) as i32},
                                  Point { x: (pos[i].x + radius[i]) as i32, y: (pos[i].y + radius[i]) as i32});

        let near_r = state.spheres.positions2.query(&Query::rect(r));


        let mut ids : Vec::<usize> = near_r.iter().map(|e_id| state.spheres.id_to_index(**e_id)).collect();

        ids.sort();

        //for j in (i + 1)..count {
        for j in ids {
            if j <= i {
                continue;
            }

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

    // reorder quadtree
    state.spheres.order_tree();
}




#[derive(Debug)]
struct ActiveSpheres {
    id_to_index : HashMap::<EntityId, usize>,
    id_to_qt_id : HashMap::<EntityId, i32>,
    positions2: QuadTree::<EntityId>,
    positions: Vec::<V3>,
    velocities: Vec::<V3>,
    radius: Vec::<f32>,
    mass: Vec::<f32>,
}

impl ActiveSpheres {

    pub fn new() -> Self {
        Self {
            id_to_index : HashMap::new(),
            id_to_qt_id : HashMap::new(),
            positions: vec![],
            velocities: vec![],
            radius: vec![],
            mass: vec![],
            positions2: QuadTree::new(Rect::from_points(Point {x: -128, y: -128}, Point { x: 128, y: 128}))
        }
    }


    pub fn id_to_index(&self, id: EntityId) -> usize {
        *self.id_to_index.get(&id).unwrap()
    }

    pub fn count(&self) -> usize {
        self.positions.len()
    }

    pub fn add_entity(&mut self, new: NewBall) -> usize {

        let bb = Rect::from_points(Point {x: (new.pos.x - new.radius) as i32, y: (new.pos.y - new.radius) as i32},
                                   Point {x: (new.pos.x + new.radius) as i32, y: (new.pos.y + new.radius) as i32});

        let qt_id = self.positions2.insert(new.id, bb);
        self.id_to_qt_id.insert(new.id, qt_id);



        let index = self.positions.len();

        self.id_to_index.insert(new.id, index);
        self.positions.push(new.pos);
        self.velocities.push(new.vel);
        self.radius.push(new.radius);
        self.mass.push(new.mass);
        index
    }

    pub fn order_tree(&mut self) {



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
