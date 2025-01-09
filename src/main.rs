use macroquad::prelude::*;

use miniquad::window::set_window_size;
use ::rand::*;

use std::process::exit;
use std::thread;
use std::time::{Duration};

const CANVAS_WIDTH : u32 = 800;
const CANVAS_HEIGHT : u32 = 450;
const NUMBER_BOIDS : u32 = 100;

struct Boid {
    x: f32,
    y: f32,
    width: f32,
    height: f32,
    dx: f32,
    dy: f32,
    color : Color
}

impl Boid {
    pub fn create() -> Boid {
        Boid {
            x: 100.0, //random::<f32>() * CANVAS_WIDTH as f32,
            y: 100.0, //random::<f32>() * CANVAS_HEIGHT as f32,
            width: 10.0,
            height: 10.0,
            dx: random::<f32>() * 5.0 - 2.5,
            dy: random::<f32>() * 5.0 - 2.5,
            color : RED
        }
    }

    pub fn update(&mut self) {
        if self.x + self.dx + self.width > CANVAS_WIDTH as f32 && self.dx > 0.0 {self.dx = -self.dx;}
        if self.y + self.dy + self.height > CANVAS_HEIGHT as f32 && self.dy > 0.0 {self.dy = -self.dy;}
        if self.x + self.dx < 0.0 && self.dx < 0.0 {self.dx = -self.dx;}
        if self.y + self.dy < 0.0 && self.dy < 0.0 {self.dy = -self.dy;}

        self.x += self.dx;
        self.y += self.dy;
    }

    
}

#[macroquad::main("MyGame")]
async fn main(){

    set_window_size(CANVAS_WIDTH, CANVAS_HEIGHT);

    let mut boids = (0..NUMBER_BOIDS).into_iter().map(|_| Boid::create()).collect::<Vec<_>>();

    loop {
        clear_background(WHITE);

        for boid in boids.iter_mut() {
            boid.update();
            draw_rectangle(boid.x, boid.y, boid.width, boid.height, boid.color);

        }

        next_frame().await
    }
    
}
