use nalgebra::vector;
use nalgebra as na;
use std::time::Instant;
use gl_lib::{gl, helpers, objects::{sphere, cube}, shader::{self, Shader}, camera};
use gl_lib::sdl2::keyboard::Keycode;
use gl_lib::controller;


use crate::*;

pub fn run_with_render(mut state: sim::State) {

    // setup render
    let sdl_setup = helpers::setup_sdl().unwrap();
    let window = sdl_setup.window;
    let sdl = sdl_setup.sdl;
    let gl = &sdl_setup.gl;

    // use sphere for now


    let height = sdl_setup.height;
    let width = sdl_setup.width;

    let mut camera = camera::Camera::new(width as f32, height as f32);

    camera.set_zfar(500.0);
    let sphere_shader = create_sphere_shader(&gl);
    let cube_shader = create_cube_shader(&gl);

    let sphere = sphere::Sphere::new(&gl, 10, 10);

    let cube = cube::Cube::new(&gl);


    // Set background color to white
    // enable depth testing
    unsafe {
        gl.ClearColor(1.0, 1.0, 1.0, 1.0);
        gl.Enable(gl::DEPTH_TEST);
    }


    camera.move_to(na::Vector3::new(0.0, 40.0, 0.0));
    camera.look_at(na::Vector3::new(-6.0, 0.0, 0.0));

    let mut instant = Instant::now();
    let mut accumulator = 0.0;
    let sim_step_time = 0.01;
    let speed = 1.0;



    let colors = vec![sim::V3::new(1.0, 0.0, 0.0),
                      sim::V3::new(0.0, 1.0, 0.0),
                      sim::V3::new(0.0, 0.0, 1.0)];

    let light_pos = na::Vector3::new(0.0, 0.0, 5.0);

    sphere_shader.set_vec3(gl, "lightPos", light_pos);
    sphere_shader.set_vec3(gl, "lightColor", na::Vector3::new(1.0, 1.0, 1.0));

    cube_shader.set_vec3(gl, "lightPos", light_pos);
    cube_shader.set_vec3(gl, "lightColor", na::Vector3::new(1.0, 1.0, 1.0));

    let mut camera_controller : camera::Controller = Default::default();
    let mut event_pump = sdl.event_pump().unwrap();
    camera_controller.speed = 40.0;

    let mut kb_map = setup_keyboard_mapping();

    let mut kb_state = KbState { state, paused: false };

    let mut frame = 0;
    loop {

        println!("{:?}", kb_state.state.spheres.positions);
        if frame > 100 {

            //break;
        }
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
            /*
            // Simulation part
            while accumulator >= sim_step_time {
                //sim::step(&mut kb_state.state, sim_step_time);
                accumulator -= sim_step_time;
            }
            */
            // simulate 1 step per frame, otherwise we might be too slow, if we cannot keep up
            sim::step(&mut kb_state.state, sim_step_time);
        }

        // Rendering
        unsafe {
            gl.Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
        }


        let ri = RenderInfo {
            gl,
            sphere_shader: & sphere_shader,
            sphere: &sphere,
            camera: &camera,
            colors: &colors,
            cube: &cube,
            cube_shader: &cube_shader

        };

        render_spheres(&kb_state.state, &ri);
        render_walls(&kb_state.state, &ri);

        window.gl_swap_window();

        frame +=1;


    }
}


struct RenderInfo<'a> {
    gl: &'a gl::Gl,
    sphere_shader: &'a shader::BaseShader,
    cube_shader: &'a shader::BaseShader,
    sphere: &'a sphere::Sphere,
    cube: &'a cube::Cube,
    camera: &'a camera::Camera,
    colors: &'a Vec::<sim::V3>
}

fn render_spheres(state: &sim::State, ri: &RenderInfo) {

    ri.sphere_shader.set_mat4(ri.gl, "view", ri.camera.view());
    ri.sphere_shader.set_mat4(ri.gl, "projection", ri.camera.projection());

    let radius = state.sphere_radius();
    let mut i = 0;
    // Render each Sphere
    for pos in state.sphere_positions() {

        ri.sphere_shader.set_vec3(ri.gl, "color", ri.colors[i % 3]);
        let model_mat =  na::Matrix4::new_translation(&pos);
        ri.sphere_shader.set_mat4(ri.gl, "model", model_mat);
        ri.sphere_shader.set_f32(ri.gl, "radius", radius[i]);
        ri.sphere.render(ri.gl);
        i +=1;
    }
}

fn render_walls(state: &sim::State, ri: &RenderInfo) {

    ri.cube_shader.set_mat4(ri.gl, "view", ri.camera.view());
    ri.cube_shader.set_mat4(ri.gl, "projection", ri.camera.projection());

    let mut i = 0;
    // Render each Sphere

    for pos in &state.walls.positions {
        ri.cube_shader.set_vec3(ri.gl, "color", ri.colors[i % 3]);
        let mut model_mat = na::Matrix4::identity();
        model_mat = model_mat.prepend_nonuniform_scaling(&state.walls.sizes[i]);
        model_mat =model_mat.append_translation(pos);
        ri.cube_shader.set_mat4(ri.gl, "model", model_mat);
        ri.cube.render(ri.gl);
        i +=1;
    }

}


fn setup_keyboard_mapping() -> controller::ControllerMapping<KbState> {
    let mut kb_map = controller::ControllerMapping::new();

    use Keycode::*;
    kb_map.exit(Keycode::Escape);
    kb_map.add_on_press(Keycode::R, reset);
    kb_map.add_on_press(P, pause);
    kb_map.add_on_press(K, dump);

    kb_map
}



struct KbState {
    pub state: sim::State,
    pub paused: bool
}

fn reset(state: &mut KbState) {
    state.state = collision();
}

fn pause(state: &mut KbState) {
    state.paused = !state.paused;
}

fn dump(state: &mut KbState) {
    state.state.dump("state.txt");
}




fn create_sphere_shader(gl: &gl::Gl) -> shader::BaseShader {
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



fn create_cube_shader(gl: &gl::Gl) -> shader::BaseShader {
    let vert_source = r"#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;


out VS_OUTPUT {
   vec3 FragPos;
   flat vec3 Normal;
} OUT;


uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
    OUT.Normal = normalize(aPos);
    vec4 pos = vec4(aPos,1.0);

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
