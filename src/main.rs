use std::{collections::HashMap, ops::Div};

use macroquad::prelude::*;

use miniquad::window::set_window_size;

use ::rand::*;
use rand_distr::{Normal, Distribution};

const CANVAS_WIDTH : u32 = 1400;
const CANVAS_HEIGHT : u32 = 700;
const NUMBER_BOIDS : u32 = 1000;
const NUMBER_SQUARE_OBSTACLES: u32 = 35;
const BOID_WIDTH : f32 = 5.0;
const MARGIN : f32 = 25.0;
const OBSTACLE_MARGIN: f32 = 10.0;
const LEFT_MARGIN : f32 = MARGIN * BOID_WIDTH;
const RIGHT_MARGIN : f32 = CANVAS_WIDTH as f32 - MARGIN * BOID_WIDTH;
const TOP_MARGIN : f32 = MARGIN * BOID_WIDTH;
const BOTTOM_MARGIN : f32 = CANVAS_HEIGHT as f32 - MARGIN * BOID_WIDTH;
const TURNING_FACTOR : f32 = 0.25;
const SPEED : f32 = 3.0;
const SEGMENT_WIDTH : u32 = 25;
const VISUAL_RANGE : f32 = 40.0;
const SPACE_RANGE : f32 = 10.0;
const CENTERING : f32 = 0.0005;
const AVOID : f32 = 0.05;
const MATCHING : f32 = 0.05;
const OBSTACLE_MIN_RADIUS: f32 = 10.0;
const OBSTACLE_MAX_RADIUS: f32 = 25.0;


struct Boid {
    x: f32, // actual x
    y: f32, // actual y
    xz: f32, // what robot thinks is x
    yz: f32, // what robot thinks is y
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

struct Obstacle {
    x: f32,
    y: f32,
    radius: f32
}

impl Obstacle {

    pub fn create() -> Obstacle {
        let w = thread_rng().gen_range(OBSTACLE_MIN_RADIUS..OBSTACLE_MAX_RADIUS);
        Obstacle {
            x: thread_rng().gen_range(CANVAS_WIDTH as f32 / 5.0..4.0 * CANVAS_WIDTH as f32 / 5.0),
            y: thread_rng().gen_range(CANVAS_HEIGHT as f32 / 5.0..4.0 * CANVAS_HEIGHT as f32 / 5.0),
            radius: w
        }
    }

    pub fn create_spec(x: f32, y: f32, radius: f32) -> Obstacle {
        Obstacle {
            x: x,
            y: y,
            radius: radius
        }
    }

    pub fn margins(&self, point: &Boid) -> Option<f32> {
        let dist_center = ((point.x + point.width / 2.0 - self.x).powi(2) + (point.y + point.width / 2.0 - self.y).powi(2)).sqrt();
        if dist_center < self.radius + VISUAL_RANGE {
            return Some(dist_center - self.radius);
        }

        return None;
    }

    pub fn draw(&self) {
        draw_circle(self.x, self.y, self.radius, BLACK);
    }
}

impl Boid {
    pub fn create() -> Boid {
        let x = thread_rng().gen_range(CANVAS_WIDTH as f32 / 5.0..4.0 * CANVAS_WIDTH as f32 / 5.0);
        let y = thread_rng().gen_range(CANVAS_HEIGHT as f32 / 5.0..4.0 * CANVAS_HEIGHT as f32 / 5.0);
        Boid {
            x: x,
            y: y,
            xz: x,
            yz: y,
            width: BOID_WIDTH,
            height: BOID_WIDTH,
            dx: thread_rng().gen_range(-2.5..=2.5),
            dy: thread_rng().gen_range(-2.5..=2.5),
            color : RED
        }
    }

    pub fn update(&mut self, segments: &HashMap<Segment, Vec<BoidPosVel>>, obstacles: &Vec<Obstacle>, normal_dist_speed: &Normal<f32>) -> Segment {
        // optimization of checks: use actual coordinates
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
                    // use noisy coordinates here
                    let dx = self.xz - point.x;
                    let dy = self.yz - point.y;

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

        // use noisy coordinates
        if self.xz + self.dx + self.width > RIGHT_MARGIN && self.dx > 0.0 {self.dx -= TURNING_FACTOR;}
        if self.yz + self.dy + self.height > BOTTOM_MARGIN && self.dy > 0.0 {self.dy -= TURNING_FACTOR;}
        if self.xz + self.dx < LEFT_MARGIN && self.dx < 0.0 {self.dx += TURNING_FACTOR;}
        if self.yz + self.dy < TOP_MARGIN && self.dy < 0.0 {self.dy += TURNING_FACTOR;}

        for obs in obstacles.iter() {

            let margins = obs.margins(&self);
            
            if margins.is_none() {continue;}
            let result = margins.unwrap();
            if result < OBSTACLE_MARGIN {
                // use noisy coordinates
                self.dx = -1.0 * (obs.x - self.xz);
                self.dy = -1.0 * (obs.y - self.yz);
            } else {
                self.dx += -1.0 * (obs.x - self.xz) * AVOID * TURNING_FACTOR; // TODO switch this up with first?
                self.dy += -1.0 * (obs.y - self.yz) * AVOID * TURNING_FACTOR;
            }

        }

        // noisy movement with ideal speed actuation
        let speed = (self.dx.powi(2) + self.dy.powi(2)).sqrt();

        self.xz += (self.dx / speed) * SPEED;
        self.yz += (self.dy / speed) * SPEED;

        // actual movement with noise in speed actuation

        let dx_samp = normal_dist_speed.sample(&mut thread_rng());
        let dy_samp = normal_dist_speed.sample(&mut thread_rng());

        let speed_actual = ((self.dx + dx_samp).powi(2) + (self.dy + dy_samp).powi(2)).sqrt();

        self.dx = ((self.dx + dx_samp) / speed_actual) * SPEED;
        self.dy = ((self.dy + dy_samp) / speed_actual) * SPEED;

        self.x += self.dx;
        self.y += self.dy;

        // use actual coordinates to keep within the bounds

        if self.x + self.width > CANVAS_WIDTH as f32 {self.x = CANVAS_WIDTH as f32 - self.width - 1.0;}
        if self.y + self.height > CANVAS_HEIGHT as f32 {self.y = CANVAS_HEIGHT as f32 - self.height - 1.0;}
        if self.x < 0.0 {self.x = 1.0;}
        if self.y < 0.0 {self.y = 1.0;}

        // optimization of checks: use actual coordinates

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
    let obstacles = (0..NUMBER_SQUARE_OBSTACLES).into_iter().map(|_| Obstacle::create()).collect::<Vec<_>>();

    let normal_dist_speed= Normal::new(0.0, SPEED / 3.0).unwrap();

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
            pos_segments.entry(boid.update(&prev_pos_segments, &obstacles, &normal_dist_speed))
                .or_insert_with(Vec::new)
                .push(BoidPosVel{x: boid.x, y: boid.y, dx: boid.dx, dy: boid.dy});
            boid.draw();
        }

        for obs in obstacles.iter() {
            obs.draw();
        }

        prev_pos_segments = pos_segments;

        next_frame().await
    }
    
}
