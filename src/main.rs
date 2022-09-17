mod simulation;
use simulation as sim;
use nalgebra::vector;
use nalgebra as na;
use std::time::Instant;
use gl_lib::{gl, helpers, objects::sphere, shader::{self, Shader}, camera};
use gl_lib::sdl2::keyboard::Keycode;
use gl_lib::controller;

fn main() {
    //multiple();
    let state = collision();
    run_with_render(state);
}


fn collision() -> sim::State {
    // wait for input

    //let mut line = String::new();
    //std::io::stdin().read_line(&mut line).unwrap(); // including '\n'

    let mut state = sim::State::new();

    state.add_ball(vector![-9.0, 0.0, 0.0], vector![5.0,0.0,0.0], 1.0, 2.0);

    state.add_ball(vector![0.0, 0.0, 0.0], vector![0.0,0.2,0.0], 0.5, 0.1);

    state.add_ball(vector![5.0, 0.0, 0.0], vector![-2.5,0.0,0.0], 2.5, 4.0);

    state
/*    let mut state = sim::State::new();

    add_grid(10,10, &mut state);
*/


}

fn run_with_render(mut state: sim::State) {

    // setup render
    let sdl_setup = helpers::setup_sdl().unwrap();
    let window = sdl_setup.window;
    let sdl = sdl_setup.sdl;
    let gl = &sdl_setup.gl;

    // use sphere for now


    let height = sdl_setup.height;
    let width = sdl_setup.width;

    let mut camera = camera::Camera::new(width as f32, height as f32);
    let shader = create_shader(&gl);
    let sphere = sphere::Sphere::new(&gl, 10, 10);


    // Set background color to white
    // enable depth testing
    unsafe {
        gl.ClearColor(1.0, 1.0, 1.0, 1.0);
        gl.Enable(gl::DEPTH_TEST);
    }


    camera.move_to(na::Vector3::new(10.0, 10.0, 1.0));
    camera.look_at(na::Vector3::new(0.0, 0.0, 0.0));

    let mut instant = Instant::now();
    let mut accumulator = 0.0;
    let sim_step_time = 0.01;
    let speed = 1.0;



    let colors = vec![sim::V3::new(1.0, 0.0, 0.0),
                      sim::V3::new(0.0, 1.0, 0.0),
                      sim::V3::new(0.0, 0.0, 1.0)];

    let light_pos = na::Vector3::new(0.0, 0.0, 5.0);

    shader.set_vec3(gl, "lightPos", light_pos);
    shader.set_vec3(gl, "lightColor", na::Vector3::new(1.0, 1.0, 1.0));

    let mut camera_controller : camera::Controller = Default::default();
    let mut event_pump = sdl.event_pump().unwrap();
    camera_controller.speed = 10.0;

    let mut kb_map = setup_keyboard_mapping();

    let mut kb_state = KbState { state, paused: false };

    loop {

        for event in event_pump.poll_iter() {
            // maybe return consumed
            camera_controller.update_events(&sdl.mouse(), event.clone());
            controller::on_input(event, &kb_map, &mut kb_state);
        }

        let delta = (instant.elapsed().as_millis() as f32) / 1000.0;
        instant = Instant::now();

        camera_controller.update_camera(&mut camera, delta);


        if !kb_state.paused {



            accumulator += delta * speed;

            // Simulation part
            while accumulator >= sim_step_time {
                sim::step(&mut kb_state.state, sim_step_time);
                accumulator -= sim_step_time;
            }
        }

        // Rendering
        unsafe {
            gl.Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
        }

        shader.set_mat4(&gl, "view", camera.view());
        shader.set_mat4(&gl, "projection", camera.projection());



        let mut i = 0;
        // Render each Sphere
        for pos in &kb_state.state.active_entities.positions {

            shader.set_vec3(&gl, "color", colors[i % 3]);
            let model_mat = na::Matrix4::new_translation(pos);
            shader.set_mat4(&gl, "model", model_mat);
            shader.set_f32(&gl, "radius", kb_state.state.active_entities.radius[i]);
            sphere.render(gl);
            i +=1;
        }



        window.gl_swap_window();

    }
}



fn setup_keyboard_mapping() -> controller::ControllerMapping<KbState> {
    let mut kb_map = controller::ControllerMapping::new();

    use Keycode::*;
    kb_map.exit(Keycode::Escape);
    kb_map.add_on_press(Keycode::R, reset);
    kb_map.add_on_press(P, pause);

    kb_map
}



struct KbState {
    state:  sim::State,
    paused: bool
}

fn reset(state: &mut KbState) {
    state.state = collision();
}

fn pause(state: &mut KbState) {
    state.paused = !state.paused;
}




fn create_shader(gl: &gl::Gl) -> shader::BaseShader {
    let vert_source = r"#version 330 core
layout (location = 0) in vec3 aPos;

out VS_OUTPUT {
   vec3 FragPos;
   flat vec3 Normal;
} OUT;


uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform float radius;

void main()
{
    OUT.Normal = aPos; //  pos is normal since mesh is unit sphere
    vec4 pos = vec4(radius * aPos,1.0);

    OUT.FragPos = vec3(model * pos);
    gl_Position =  projection * view * model * pos;
}";

    let frag_source = r"#version 330 core
out vec4 FragColor;
uniform vec3 color;


uniform vec3 lightColor;
uniform vec3 lightPos;
uniform vec3 viewPos;


in VS_OUTPUT {
   vec3 FragPos;
   flat vec3 Normal;
} IN;

void main()
{
  vec3 col = color;

  // ABIENT
  float ambientStrength = 0.5;
  vec3 ambient = ambientStrength * lightColor;


  //DIFFUSE
  vec3 norm = normalize(IN.Normal);
  vec3 lightDir = normalize(lightPos - IN.FragPos);
  float diff = max(dot(norm, lightDir), 0.0);

  vec3 diffuse = (diff * lightColor) * 0.70;


  // SPECULAR
  float specularStrength = 0.1;
  vec3 viewDir = normalize(viewPos - IN.FragPos);
  vec3 reflectionDir = reflect(-lightDir, IN.Normal);

  float spec = pow(max(dot(viewDir, reflectionDir), 0.0), 5);
  vec3 specular = specularStrength * spec * lightColor;


  FragColor = vec4( (ambient + diffuse + specular) * col, 1.0f);


}";


    shader::BaseShader::new(gl, vert_source, frag_source).unwrap()
}


fn multiple() {
    let mut state = sim::State::new();

    add_grid(10,10, &mut state);

    let mut time_inst = Instant::now();

    let mut iters = 0;

    loop {
        sim::step(&mut state, 0.01);
        iters += 1;
        if iters % 10000 == 0 {
            let iter_pr_ms = (iters as f32) / (time_inst.elapsed().as_millis() as f32);
            println!("iter/ms: {:.2?}", iter_pr_ms);
            time_inst = Instant::now();
            iters = 0
        }
    }
}


fn add_grid(x: i32, y: i32, state: &mut sim::State) {

    for x in (-x/2)..(x/2) {
        for y in (-y/2)..(y/2) {
            state.add_ball(vector![x as f32, y as f32, 0.0], vector![1.0*(y as f32), 1.0/ (x as f32),0.0], 0.1, 1.0);
        }
    }
}
