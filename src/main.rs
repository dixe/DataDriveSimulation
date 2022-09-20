mod simulation;
use simulation as sim;
use nalgebra::vector;
use nalgebra as na;
use std::time::Instant;


#[cfg(feature = "sdl")]
mod render;

fn main() {
    flamegraph();

    let mut state = collision();

    #[cfg(feature = "sdl")]
    render::run_with_render(state);
}


pub fn collision() -> sim::State {
    let mut state = sim::State::new();

    add_grid(5, 5, &mut state);

    state.add_ball(vector![-100.0, 0.0, 0.0], vector![30.0,0.0,0.0], 1.0, 2.0);

    state.add_ball(vector![0.0, 10.0, 0.0], vector![0.0,0.0,0.0], 0.5, 0.1);

    state.add_ball(vector![5.0, 0.0, 0.0], vector![-2.5,0.0,0.0], 2.5, 4.0);


    state.add_ball(vector![70.0, 0.0, 0.0], vector![0.0, 0.0, 0.0], 1.5, 2.0);

    state.add_wall(vector![-10.0, 0.0, 0.0], vector![1.0, 5.0, 10.0]);

    state


}

fn wall_test() -> sim::State {

    let mut state = sim::State::new();

    state.add_ball(vector![-5.0, 0.0, 0.0], vector![5.0,0.0,0.0], 1.0, 2.0);

    state.add_wall(vector![0.0, 0.0, 0.0], vector![1.0, 2.0, 10.0]);

    state.add_wall(vector![-7.0, 0.0, 0.0], vector![1.0, 2.0, 10.0]);

    state
}


fn flamegraph() {
    let mut state = sim::State::new();

    add_grid(30, 10, &mut state);

    let mut time_inst = Instant::now();

    let mut iters = 0;

    let mut time_total = Instant::now();
    loop {
        sim::step(&mut state, 0.01);
        iters += 1;
        if time_inst.elapsed().as_millis() as f32 > 1000.0 {
            let iter_pr_ms = (iters as f32) / (time_inst.elapsed().as_millis() as f32);
            println!("iter/ms: {:.2?}", iter_pr_ms);
            time_inst = Instant::now();
            iters = 0
        }
        if time_total.elapsed().as_secs() > 10 {
            println!("Stopping");
            break;

        }


    }
}


fn add_grid(x: i32, y: i32, state: &mut sim::State) {

    for x in (-x/2)..(x/2) {
        for y in (-y/2)..(y/2) {
            //state.add_ball(vector![x as f32, y as f32, 0.0], vector![ 0.0, 0.0, 0.0], 0.1, 0.001);
            state.add_ball(vector![x as f32, y as f32, 0.0], vector![1.0*(y as f32), 1.0/ (x as f32),0.0], 0.1, 1.0);
        }
    }
}
