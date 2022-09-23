mod simulation;
use simulation as sim;
use nalgebra::vector;
use nalgebra as na;
use std::time::Instant;


#[cfg(feature = "sdl")]
mod render;

fn main() {
    #[cfg(not(feature = "sdl"))]
    flamegraph();

    let mut state = collision();

    #[cfg(feature = "sdl")]
    render::run_with_render(state);
}


pub fn collision() -> sim::State {
    let mut state = sim::State::new();

    //add_grid(10, 30, &mut state);


    //state.add_ball(vector![30.0, 0.0, 2.0], vector![-150.0,0.0,0.0], 1.0, 0.5);

    state.add_ball(vector![30.0, 0.0, 0.0], vector![-200.0,0.0,0.0], 1.0, 0.5);

    //state.add_ball(vector![30.0, 0.0, -2.0], vector![-500.0,0.0,0.0], 1.0, 0.5);


    state.add_wall(vector![0.0, 0.0, 0.0], vector![1.0, 5.0, 10.0]);

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

    add_grid(100, 100, &mut state);

    let mut time_inst = Instant::now();

    let mut iters = 0;

    let mut time_total = Instant::now();

    loop {
        sim::step(&mut state, 0.01);
        iters += 1;
        if time_inst.elapsed().as_millis() as f32 > 1000.0 {
            let ms_pr_iter = (time_inst.elapsed().as_millis() as f32) / (iters as f32) ;
            println!("ms/iter: {:.2?}", iters);
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

    state.add_wall(vector![-128.0, 0.0, 0.0], vector![10.0, 256.0, 10.0]);

    state.add_wall(vector![128.0, 0.0, 0.0], vector![10.0, 256.0, 10.0]);

    state.add_wall(vector![0.0, -128.0, 0.0], vector![256.0, 10.0, 10.0]);

    state.add_wall(vector![0.0, 128.0, 0.0], vector![256.0, 10.0, 10.0]);

    for x in (-x/2)..(x/2) {
        for y in (-y/2)..(y/2) {
            //state.add_ball(vector![x as f32, y as f32, 0.0], vector![ 0.0, 0.0, 0.0], 0.1, 0.001);
            state.add_ball(vector![x as f32, y as f32, 0.0], vector![1.0*(y as f32), 1.0/ f32::max(x as f32, 1.0), 0.0], 1.0, 1.0);
        }
    }
}
