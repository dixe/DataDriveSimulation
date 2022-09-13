use std::collections::HashMap;
use nalgebra as na;

pub type EntityId = usize;
pub type V3 = na::Vector3::<f32>;

// All into regarding the simulation
#[derive(Debug)]
pub struct State {
    next_id: EntityId,
    pub active_entities: ActiveElements,
}


impl State {
    pub fn new() -> Self {
        Self {
            next_id: 1,
            active_entities: ActiveElements::new()
        }
    }


    pub fn add_ball(&mut self, pos: V3, vel: V3, r: f32, mass: f32) -> EntityId {
        let id = self.next_id;
        self.next_id += 1;
        self.active_entities.add_entity(NewBall {
            id,
            pos,
            vel,
            radius: r,
            mass
        });

        id
    }
}


#[derive(Default, Clone, Debug)]
struct Manifold {
    vel_change : V3,
    pos_correction: V3,
}



fn impulse_manifolds(state: &mut State) -> Vec::<Manifold> {

    // parially from
    //https://gamedevelopment.tutsplus.com/tutorials/how-to-create-a-custom-2d-physics-engine-the-basics-and-impulse-resolution--gamedev-6331

    let count = state.active_entities.count();

    let mut res : Vec::<Manifold>= vec![Default::default(); count];

    let pos = &state.active_entities.positions;
    let vel = &state.active_entities.velocities;
    let radius = & state.active_entities.radius;
    let mass = & state.active_entities.mass;

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


pub fn step(state: &mut State, dt: f32) {

    let count = state.active_entities.count();
    // get acceleration of each ball, calculated from collision
    let manifolds = impulse_manifolds(state);

    //
    let pos = &mut state.active_entities.positions;
    let vel = &mut state.active_entities.velocities;

    for i in 0..count {
        vel[i] += manifolds[i].vel_change;
        pos[i] += vel[i] * dt + manifolds[i].pos_correction;

    }

}




#[derive(Debug)]
pub struct ActiveElements {
    id_to_index : HashMap::<EntityId,usize>,
    pub positions: Vec::<V3>,
    pub velocities: Vec::<V3>,
    pub radius: Vec::<f32>,
    pub mass: Vec::<f32>,
}

impl ActiveElements {

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
