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



fn impulse_manifolds(state: &mut State) {

    // parially from
    //https://gamedevelopment.tutsplus.com/tutorials/how-to-create-a-custom-2d-physics-engine-the-basics-and-impulse-resolution--gamedev-6331

    let count = state.spheres.count();

    //let mut res : Vec::<Manifold>= vec![Default::default(); count];

   let pos = &state.spheres.positions;
    let vel = &state.spheres.velocities;
    let radius = &state.spheres.radius;
    let mass = &state.spheres.mass;

    let mut query_res = vec![];
    let mut ids : Vec::<usize> = vec![];

    for i in 0..count {

        query_res.clear();
        ids.clear();

        let r = Rect::from_points(Point { x: (pos[i].x - radius[i]) as i32, y: (pos[i].y - radius[i]) as i32},
                                  Point { x: (pos[i].x + radius[i]) as i32, y: (pos[i].y + radius[i]) as i32});

        state.spheres.positions2.query(r, &mut query_res);

        for &q_id in &query_res {
            ids.push(state.spheres.qt_id_to_index(q_id))
        }
        //let mut ids : Vec::<usize> = near_element_ids.iter().map(|e_id| state.spheres.qt_id_to_index(*e_id)).collect();

        //ids.sort();

        //for j in (i + 1)..count {


        for &j in &ids {
            if j <= i{
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
                let resitution = 0.8; // elasicity e = rel_vel_after_col / rel_vel_before, so 1 is all energy preserved,

                let mut impulse_scalar = -(1.0 + resitution) * vel_along_norm;
                impulse_scalar /= 1.0/mass[i] + 1.0/mass[j];

                let impulse : V3 = col_norm * impulse_scalar;


                state.spheres.manifolds[i].vel_change -= 1.0/mass[i] * impulse;
                state.spheres.manifolds[j].vel_change += 1.0/mass[j] * impulse;

                let percent = 0.1; // between 0.2 and 0.8 usually
                let correction : V3 = pen_depth / (1.0/mass[i] + 1.0/mass[j]) * percent * col_norm;

                state.spheres.manifolds[i].pos_correction -= 1.0/mass[i] * correction;
                state.spheres.manifolds[j].pos_correction += 1.0/mass[j] * correction;


            }
        }
    }

    //res
}



fn impulse_walls(state: &mut State) {

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
                state.spheres.manifolds[i].vel_change -= 1.0/mass[i] * impulse;

                let percent = 0.1; // between 0.2 and 0.8 usually

                let pen_depth = radius[i] - dist;
                let correction : V3 = (pen_depth / 1.0/mass[i]) * percent * normal;

                state.spheres.manifolds[i].pos_correction -= 1.0/mass[i] * correction;

            }
        }
    }
}


pub fn step(state: &mut State, dt: f32) {

    let count = state.spheres.count();
    // get acceleration of each ball, calculated from collision

    for i in 0..count {
        state.spheres.manifolds[i].vel_change.x = 0.0;
        state.spheres.manifolds[i].vel_change.y = 0.0;
        state.spheres.manifolds[i].vel_change.z = 0.0;

        state.spheres.manifolds[i].pos_correction.x = 0.0;
        state.spheres.manifolds[i].pos_correction.y = 0.0;
        state.spheres.manifolds[i].pos_correction.z = 0.0;

    }



    impulse_manifolds(state);

    impulse_walls(state);

    //
    let pos = &mut state.spheres.positions;
    let vel = &mut state.spheres.velocities;


    for i in 0..count {
        vel[i] += state.spheres.manifolds[i].vel_change;
        pos[i] += vel[i] * dt + state.spheres.manifolds[i].pos_correction;
    }

    // reorder quadtree
    state.spheres.order_tree();
}




#[derive(Debug)]
struct ActiveSpheres {
    qt_id_to_index : HashMap::<i32, usize>,
    id_to_qt_id : HashMap::<EntityId, i32>,
    positions2: QuadTree::<EntityId>,
    positions: Vec::<V3>,
    velocities: Vec::<V3>,
    radius: Vec::<f32>,
    mass: Vec::<f32>,
    manifolds: Vec::<Manifold>
}

impl ActiveSpheres {

    pub fn new() -> Self {
        let mut qt = QuadTree::new(Rect::from_points(Point {x: -128, y: -128}, Point { x: 128, y: 128}));

        qt.set_elements_per_node(6);
        Self {
            qt_id_to_index : HashMap::new(),
            id_to_qt_id : HashMap::new(),
            positions: vec![],
            velocities: vec![],
            radius: vec![],
            mass: vec![],
            manifolds: vec![],
            positions2: qt,
        }
    }


    pub fn qt_id_to_index(&self, qt_id: i32) -> usize {
        *self.qt_id_to_index.get(&qt_id).unwrap()
    }

    pub fn count(&self) -> usize {
        self.positions.len()
    }

    pub fn add_entity(&mut self, new: NewBall) -> usize {

        let bb = Rect::from_points(Point {x: (new.pos.x - new.radius).floor() as i32, y: (new.pos.y - new.radius).floor() as i32},
                                   Point {x: (new.pos.x + new.radius).ceil()as i32, y: (new.pos.y + new.radius).ceil() as i32});


        let element_id = self.positions2.insert(new.id, bb);
        let index = self.positions.len();


        self.qt_id_to_index.insert(element_id, index);

        self.positions.push(new.pos);
        self.velocities.push(new.vel);
        self.radius.push(new.radius);
        self.mass.push(new.mass);
        self.manifolds.push(Manifold {
            vel_change : vector![0.0, 0.0, 0.0],
            pos_correction: vector![0.0, 0.0, 0.0]});
        index
    }

    pub fn order_tree(&mut self) {

        let count = self.positions.len();

        let qt = &mut self.positions2;
        let qt_ids : Vec<(i32, usize)> = self.qt_id_to_index.iter().map(|(&q_id, &index)| (q_id, index)).collect();
        for &(qt_id, i) in qt_ids.iter() {

            let pos = self.positions[i];
            let radius =  self.radius[i];
            let bb = Rect::from_points(Point {x: (pos.x - radius).floor() as i32, y: (pos.y - radius).floor() as i32},
                                       Point {x: (pos.x + radius). ceil()as i32, y: (pos.y + radius).ceil() as i32});

            qt.remove(qt_id);

            // insert at new position
            let new_id = qt.insert(i, bb);

            self.qt_id_to_index.insert(new_id, i);

        }

        self.positions2.cleanup();


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
