use std::{collections::HashMap, ops::Div};

use macroquad::prelude::*;

use miniquad::window::set_window_size;
use ::rand::*;

const CANVAS_WIDTH : u32 = 1400;
const CANVAS_HEIGHT : u32 = 700;
const NUMBER_BOIDS : u32 = 1000;
const BOID_WIDTH : f32 = 5.0;
const MARGIN : f32 = 25.0;
const LEFT_MARGIN : f32 = MARGIN * BOID_WIDTH;
const RIGHT_MARGIN : f32 = CANVAS_WIDTH as f32 - MARGIN * BOID_WIDTH;
const TOP_MARGIN : f32 = MARGIN * BOID_WIDTH;
const BOTTOM_MARGIN : f32 = CANVAS_HEIGHT as f32 - MARGIN * BOID_WIDTH;
const TURNING_FACTOR : f32 = 0.25;
// const MIN_SPEED : f32 = 2.0;
const SPEED : f32 = 3.0;
const SEGMENT_WIDTH : u32 = 25;
const VISUAL_RANGE : f32 = 60.0;
const SPACE_RANGE : f32 = 10.0;
const CENTERING : f32 = 0.0005;
const AVOID : f32 = 0.05;
const MATCHING : f32 = 0.05;



struct Boid {
    x: f32,
    y: f32,
    width: f32,
    height: f32,
    dx: f32,
    dy: f32,
    color : Color
}

#[derive(Eq, Hash, PartialEq)]
struct Segment {
    x: u32,
    y: u32
}

struct BoidPosVel {
    x: f32,
    y: f32,
    dx: f32,
    dy: f32
}

impl Boid {
    pub fn create() -> Boid {
        Boid {
            x: random::<f32>() * CANVAS_WIDTH as f32 - 20.0,
            y: random::<f32>() * CANVAS_HEIGHT as f32 - 20.0,
            width: BOID_WIDTH,
            height: BOID_WIDTH,
            dx: random::<f32>() * 5.0 - 2.5,
            dy: random::<f32>() * 5.0 - 2.5,
            color : RED
        }
    }

    pub fn update(&mut self, segments: &HashMap<Segment, Vec<BoidPosVel>>) -> Segment {
        let min_seg_x = ((self.x - VISUAL_RANGE).max(0.0) as u32).div(SEGMENT_WIDTH);
        let max_seg_x = ((self.x + VISUAL_RANGE).min(CANVAS_WIDTH as f32) as u32).div(SEGMENT_WIDTH);
        let min_seg_y = ((self.y - VISUAL_RANGE).max(0.0) as u32).div(SEGMENT_WIDTH);
        let max_seg_y = ((self.y + VISUAL_RANGE).min(CANVAS_HEIGHT as f32) as u32).div(SEGMENT_WIDTH);

        let mut avoid_dx : f32 = 0.0;
        let mut avoid_dy : f32 = 0.0;
        let mut avg_x : f32 = 0.0;
        let mut avg_y : f32 = 0.0;
        let mut avg_dx : f32 = 0.0;
        let mut avg_dy : f32 = 0.0;

        let mut others : u32 = 0;

        for i in min_seg_x..=max_seg_x {
            for j in min_seg_y..=max_seg_y {
                for point in segments.get(&Segment{x: i, y: j}).unwrap_or(&Vec::new()) {
                    let dx = self.x - point.x;
                    let dy = self.y - point.y;

                    if (dx.powi(2) + dy.powi(2)).sqrt() < SPACE_RANGE {
                        avoid_dx += dx;
                        avoid_dy += dy;
                    }

                    else if (dx.powi(2) + dy.powi(2)).sqrt() < VISUAL_RANGE {
                        avg_x += point.x;
                        avg_y += point.y;
                        avg_dx += point.dx;
                        avg_dy += point.dy;

                        others += 1;
                    }
                }
            }
        }

        if others > 0 {
            let xp = avg_x / (others as f32);
            let yp = avg_y / (others as f32);
            let xv = avg_dx / (others as f32);
            let yv = avg_dy / (others as f32);

            self.dx += (xp - self.x) * CENTERING + (xv - self.dx) * MATCHING;
            self.dy += (yp - self.y) * CENTERING + (yv - self.dy) * MATCHING;
        }

        self.dx += avoid_dx * AVOID;
        self.dy += avoid_dy * AVOID;




        if self.x + self.dx + self.width > RIGHT_MARGIN && self.dx > 0.0 {self.dx = self.dx - TURNING_FACTOR;}
        if self.y + self.dy + self.height > BOTTOM_MARGIN && self.dy > 0.0 {self.dy = self.dy - TURNING_FACTOR;}
        if self.x + self.dx < LEFT_MARGIN && self.dx < 0.0 {self.dx = self.dx + TURNING_FACTOR;}
        if self.y + self.dy < TOP_MARGIN && self.dy < 0.0 {self.dy = self.dy + TURNING_FACTOR;}

        let speed = (self.dx.powi(2) + self.dy.powi(2)).sqrt();

        self.dx = (self.dx / speed) * SPEED;
        self.dy = (self.dy / speed) * SPEED;


        // if speed < MIN_SPEED {
        //     self.dx = (self.dx / speed) * MIN_SPEED;
        //     self.dy = (self.dy / speed) * MIN_SPEED;
        // }

        self.x += self.dx;
        self.y += self.dy;

        if self.x + self.width > CANVAS_WIDTH as f32 {self.x = CANVAS_WIDTH as f32 - self.width - 1.0;}
        if self.y + self.height > CANVAS_HEIGHT as f32 {self.y = CANVAS_HEIGHT as f32 - self.height - 1.0;}
        if self.x < 0.0 {self.x = 1.0;}
        if self.y < 0.0 {self.y = 1.0;}

        Segment{x: (self.x as u32).div(SEGMENT_WIDTH), y: (self.y as u32).div(SEGMENT_WIDTH)}
    }

    pub fn draw(&mut self) {
        let mut original = Vec::new();
            let mut res = Vec::new();
            original.push(Vec2{x: self.x + self.width, y: self.y + self.height / 2.0});
            original.push(Vec2{x: self.x, y: self.y});
            original.push(Vec2{x: self.x, y: self.y + self.height});

            let cx = self.x + self.width / 2.0;
            let cy = self.y + self.height / 2.0;

            let alpha = self.dy.atan2(self.dx);
            
            for point in original.iter_mut() {
                let xp = point.x - cx;
                let yp = point.y - cy;
                let xpp = xp * alpha.cos() - yp * alpha.sin();
                let ypp = xp * alpha.sin() + yp * alpha.cos();
                res.push(Vec2{x: xpp + cx, y: ypp + cy});
            }

            draw_triangle(res[0], res[1], res[2], self.color);
    }

    
}

#[macroquad::main("MyGame")]
async fn main(){

    set_window_size(CANVAS_WIDTH, CANVAS_HEIGHT);

    let mut boids = (0..NUMBER_BOIDS).into_iter().map(|_| Boid::create()).collect::<Vec<_>>();

    let mut prev_pos_segments : HashMap<Segment, Vec<BoidPosVel>> = HashMap::new();

    for boid in boids.iter() {
        prev_pos_segments.entry(Segment{x: (boid.x as u32).div(SEGMENT_WIDTH), y: (boid.y as u32).div(SEGMENT_WIDTH)})
            .or_insert_with(Vec::new)
            .push(BoidPosVel{x: boid.x, y: boid.y, dx: boid.dx, dy: boid.dy});
    }

    loop {
        clear_background(WHITE);

        let mut pos_segments : HashMap<Segment, Vec<BoidPosVel>> = HashMap::new();

        for boid in boids.iter_mut() {
            pos_segments.entry(boid.update(&prev_pos_segments))
                .or_insert_with(Vec::new)
                .push(BoidPosVel{x: boid.x, y: boid.y, dx: boid.dx, dy: boid.dy});
            boid.draw();
        }

        prev_pos_segments = pos_segments;

        next_frame().await
    }
    
}
