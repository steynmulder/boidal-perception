use std::{collections::HashMap, fs::{File, OpenOptions}, io::{Read, Write}, ops::Div, path::Path, time::SystemTime};

use nalgebra::{DMatrix, DVector, Matrix};

use macroquad::prelude::*;

use miniquad::window::set_window_size;

use ::rand::*;
use rand_distr::{Normal, Distribution};

enum positioning{WLS, EKF}

const MINIMUM_FRAME_TIME: f32 = 1. / 30.;
const CANVAS_WIDTH : u32 = 750;
const CANVAS_HEIGHT : u32 = 450;
const NUMBER_BOIDS : u32 = 200;
const NUMBER_OBSTACLES: u32 = 50;
const BOID_WIDTH : f32 = 5.0;
const MARGIN : f32 = 25.0;
const OBSTACLE_MARGIN: f32 = 10.0;
const LEFT_MARGIN : f32 = 50.0;
const RIGHT_MARGIN : f32 = CANVAS_WIDTH as f32 - 50.0;
const TOP_MARGIN : f32 = 50.0;
const BOTTOM_MARGIN : f32 = CANVAS_HEIGHT as f32 - 50.0;
const TURNING_FACTOR : f32 = 0.5;
const SPEED : f32 = 2.0;
const SEGMENT_WIDTH : u32 = 25;
const VISUAL_RANGE : f32 = 40.0;
const ANCHOR_VISUAL_RANGE: f32 = 80.0;
const ESCAPE_VISUAL_RANGE: f32 = 15.0;
const ESCAPING_BOIDS_VISUAL_RANGE: f32 = 50.0;
const SPACE_RANGE : f32 = 10.0;
const CENTERING : f32 = 0.0005;
const AVOID : f32 = 0.01;
const MATCHING : f32 = 0.05;
const ESCAPE_FACTOR: f32 = 1.0;
const FOLLOW_ESCAPE: f32 = 0.05;
const OBSTACLE_MIN_RADIUS: f32 = 2.0;
const OBSTACLE_MAX_RADIUS: f32 = 5.0;
const ESCAPE_RADIUS: f32 = 5.0;

const NUMBER_MEASUREMENTS_CYCLE: usize = 1;

const SPEED_STD: f32 = 0.1;
const ANCHOR_STD: f32 = 0.1;

const SIM_POSITIONING: positioning = positioning::EKF;


struct Boid {
    x: f32, // actual x
    y: f32, // actual y
    theta: f32, // actual angle
    x_est: f32, // what robot thinks is x
    y_est: f32, // what robot thinks is y
    theta_est: f32, // what robot thinks is angle
    width: f32,
    height: f32,
    dx: f32,
    dy: f32,
    color : Color,
    initial: bool,
    reached: bool
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
    dy: f32,
    color: Color
}

struct Obstacle {
    x: f32,
    y: f32,
    radius: f32
}

struct Escape {
    x: f32,
    y: f32,
    radius: f32
}

impl Obstacle {

    pub fn create() -> Obstacle {
        let w = thread_rng().gen_range(OBSTACLE_MIN_RADIUS..OBSTACLE_MAX_RADIUS);
        Obstacle {
            x: thread_rng().gen_range(0.0..CANVAS_WIDTH as f32),
            y: thread_rng().gen_range(0.0..CANVAS_HEIGHT as f32),
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

impl Escape {
    pub fn create(obstacles: &Vec<Obstacle>) -> Escape {
        let mut free = false;
        let mut x = 0.0;
        let mut y = 0.0;
        let radius = ESCAPE_RADIUS;
        while !free {
            x = thread_rng().gen_range(3.0 * CANVAS_WIDTH as f32 / 8.0..5.0 * CANVAS_WIDTH as f32 / 8.0);
            y = thread_rng().gen_range(3.0 * CANVAS_HEIGHT as f32 / 8.0..5.0 * CANVAS_HEIGHT as f32 / 8.0);
            free = true;
            for obs in obstacles.iter() {
                if  ((x - obs.x).powi(2) + (y - obs.y).powi(2)).sqrt() <= radius + obs.radius {
                        free = false;
                        break;
                } 
            }
        }

        Escape {
            x: x,
            y: y,
            radius: radius
        }
    }

    pub fn draw(&mut self) {
        draw_circle(self.x, self.y, self.radius, ORANGE);
    }
}

impl Boid {
    pub fn create() -> Boid {
        let x = thread_rng().gen_range(CANVAS_WIDTH as f32 / 5.0..4.0 * CANVAS_WIDTH as f32 / 5.0);
        let y = thread_rng().gen_range(CANVAS_HEIGHT as f32 / 5.0..4.0 * CANVAS_HEIGHT as f32 / 5.0);
        Boid {
            x: x,
            y: y,
            theta: 0.0,
            x_est: x,
            y_est: y,
            theta_est: 0.0,
            width: BOID_WIDTH,
            height: BOID_WIDTH,
            dx: SPEED,
            dy: 0.0,
            color : Color{r: 1.0, g: 0.0, b: 0.0, a: 1.0},
            initial: true,
            reached: false
        }
    }

    pub fn update(&mut self,
                  segments: &HashMap<Segment, Vec<BoidPosVel>>, 
                  obstacle_segments: &HashMap<Segment, Vec<&Obstacle>>,
                  escape: &Escape,
                  normal_dist_speed: &Normal<f32>,
                  normal_dist_anchors: &Normal<f32>,
                ) -> Segment {
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
        let mut escape_x: f32 = 0.0;
        let mut escape_y: f32 = 0.0;

        let mut others : u32 = 0;

        let mut max_green: f32 = 0.0;

        if (self.x - escape.x).powi(2) + (self.y - escape.y).powi(2) <= escape.radius.powi(2) {
            self.reached = true;
            return Segment{x: 0, y: 0};
        }

        for i in min_seg_x..=max_seg_x {
            for j in min_seg_y..=max_seg_y {
                for point in segments.get(&Segment{x: i, y: j}).unwrap_or(&Vec::new()) {
                    // use noisy coordinates here
                    let dx = self.x_est - point.x;
                    let dy = self.y_est - point.y;

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

                    if (dx.powi(2) + dy.powi(2)).sqrt() < ESCAPING_BOIDS_VISUAL_RANGE && point.color.g > 0.0 {

                        if max_green < point.color.g {
                            max_green = point.color.g;
                            escape_x = point.x;
                            escape_y = point.y;
                        }
                
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

        if max_green > 0.0 {

            self.dx += (escape_x - self.x) * FOLLOW_ESCAPE;
            self.dy += (escape_y - self.y) * FOLLOW_ESCAPE;
            self.color = Color{r: 0.0, g: (max_green - 0.1).max(0.0), b: 0.0, a: 1.0};
        } else {
            self.color = Color{r: (self.color.r+0.1).min(1.0), g: (self.color.g-0.5).max(0.0), b: 0.0, a: 1.0};
        }

        self.dx += avoid_dx * AVOID;
        self.dy += avoid_dy * AVOID;

        // use noisy coordinates
        if self.x_est + self.dx + self.width > RIGHT_MARGIN && self.dx > 0.0 {self.dx -= TURNING_FACTOR;}
        if self.y_est + self.dy + self.height > BOTTOM_MARGIN && self.dy > 0.0 {self.dy -= TURNING_FACTOR;}
        if self.x_est + self.dx < LEFT_MARGIN && self.dx < 0.0 {self.dx += TURNING_FACTOR;}
        if self.y_est + self.dy < TOP_MARGIN && self.dy < 0.0 {self.dy += TURNING_FACTOR;}


        for i in min_seg_x..=max_seg_x {
            for j in min_seg_y..=max_seg_y {
                for obs in obstacle_segments.get(&Segment{x: i, y: j}).unwrap_or(&Vec::new()) {
                    let margins = obs.margins(&self);
                
                    if let Some(result) = margins {
                        if result < OBSTACLE_MARGIN {
                            // use noisy coordinates
                            self.dx = -1.0 * (obs.x - self.x_est);
                            self.dy = -1.0 * (obs.y - self.y_est);
                        } else {
                            self.dx += -1.0 * (obs.x - self.x_est) * AVOID * TURNING_FACTOR; // TODO switch this up with first?
                            self.dy += -1.0 * (obs.y - self.y_est) * AVOID * TURNING_FACTOR;
                        }

                    }

                }
                
            }
        }

        if  ((self.x - escape.x).powi(2) + (self.y - escape.y).powi(2)).sqrt() - escape.radius <= ESCAPE_VISUAL_RANGE {

                self.dx += (escape.x - self.x) * ESCAPE_FACTOR;
                self.dy += (escape.y - self.y) * ESCAPE_FACTOR;
                self.color = Color{r: 0.0, g: 1.0, b: 0.0, a: 1.0};
        }

        let min_seg_x_anchor = ((self.x - ANCHOR_VISUAL_RANGE).max(0.0) as u32).div(SEGMENT_WIDTH);
        let max_seg_x_anchor = ((self.x + ANCHOR_VISUAL_RANGE).min(CANVAS_WIDTH as f32) as u32).div(SEGMENT_WIDTH);
        let min_seg_y_anchor = ((self.y - ANCHOR_VISUAL_RANGE).max(0.0) as u32).div(SEGMENT_WIDTH);
        let max_seg_y_anchor = ((self.y + ANCHOR_VISUAL_RANGE).min(CANVAS_HEIGHT as f32) as u32).div(SEGMENT_WIDTH);

        let mut true_distances_anchors = Vec::new();
        let mut number_anchors: usize = 0;
        let mut anchors : Vec<&Obstacle> = Vec::new();

        for i in min_seg_x_anchor..=max_seg_x_anchor {
            for j in min_seg_y_anchor..=max_seg_y_anchor {
                for obs in obstacle_segments.get(&Segment{x: i, y: j}).unwrap_or(&Vec::new()) {
                    let distance = ((obs.x - self.x).powi(2) + (obs.y - self.y).powi(2)).sqrt();

                    if distance < ANCHOR_VISUAL_RANGE {
                        number_anchors += 1;
                        true_distances_anchors.push(distance);
                        anchors.push(obs);
                    }
                }
            }
        }


        if number_anchors > 2 {
            let mut noisy_distances = Vec::new();

            for _i in 0..NUMBER_MEASUREMENTS_CYCLE {
                for distance in true_distances_anchors.clone().into_iter() {
                    noisy_distances.push(distance + normal_dist_anchors.sample(&mut thread_rng()));
                }
            }

            let anchor_measurements: DMatrix<f32> = DMatrix::from_row_slice(NUMBER_MEASUREMENTS_CYCLE, number_anchors, &noisy_distances);
            
            let mut h: DMatrix<f32> = DMatrix::zeros((number_anchors - 1) * NUMBER_MEASUREMENTS_CYCLE, 2);
            let mut z: DMatrix<f32> = DMatrix::zeros((number_anchors - 1) * NUMBER_MEASUREMENTS_CYCLE, 1);
            let mut c: DMatrix<f32> = DMatrix::zeros((number_anchors - 1) * NUMBER_MEASUREMENTS_CYCLE, (number_anchors - 1) * NUMBER_MEASUREMENTS_CYCLE);

            for i in 0..NUMBER_MEASUREMENTS_CYCLE {
                let l = i * (number_anchors - 1);
                self.trilateration(&anchors, &anchor_measurements, &mut h, &mut z, &mut c, &i, &l, &number_anchors);
            }

            let p = (&h.transpose() * &c.clone_owned().try_inverse().unwrap() * &h).try_inverse();
            let c_inv = c.clone().try_inverse();
            if p.is_none() || c_inv.is_none() {
                let speed = (self.dx.powi(2) + self.dy.powi(2)).sqrt();

                self.x_est += (self.dx / speed) * SPEED;
                self.y_est += (self.dy / speed) * SPEED;
                self.theta_est = self.dy.atan2(self.dx);
            } else {
                match SIM_POSITIONING {
                    positioning::WLS => {
                        let x_ls = &p.clone().unwrap() * &h.transpose() * c_inv.unwrap() * &z;
                        self.x_est = x_ls[0];
                        self.y_est = x_ls[1];
                        self.theta_est = self.dy.atan2(self.dx);
                    },
                    positioning::EKF => {
                        if self.initial {
                            self.initial = false;
                            let x_ls = &p.clone().unwrap() * &h.transpose() * c_inv.unwrap() * &z;
                            self.x_est = x_ls[0];
                            self.y_est = x_ls[1];
                            self.theta_est = self.dy.atan2(self.dx);
                        } else {
                            let mut p_vals = p.clone().unwrap().insert_columns(2, 1, 0.0).insert_rows(2, 1, 0.0);
                            p_vals[(2,2)] = ANCHOR_STD;

                            // predication
                            let x_pred = self.dynamic(self.dy.atan2(self.dx) - self.theta_est);
                            let a_c = DMatrix::from_row_slice(3, 3, &[1., 0., -SPEED * self.theta_est.sin() * MINIMUM_FRAME_TIME,
                                                                                                                     0., 1., SPEED * self.theta_est.cos() * MINIMUM_FRAME_TIME,
                                                                                                                     0., 0., 1.]);
                            let p_pred = &a_c * &p_vals * &a_c.transpose();

                            // update
                            let h_pad = h.insert_column(2, 0.); //todo check if correct column
                            let k_1 = &p_pred * &h_pad.transpose();
                            let k_2 = (&h_pad * &p_pred * &h_pad.transpose() + &c).try_inverse();
                            if k_2.is_none() {
                                println!("Something went wrong");
                                let speed = (self.dx.powi(2) + self.dy.powi(2)).sqrt();

                                self.x_est += (self.dx / speed) * SPEED;
                                self.y_est += (self.dy / speed) * SPEED;
                                self.theta_est = self.dy.atan2(self.dx);
                            } else {
                                let k = k_1 * k_2.unwrap();
                                let x_next = &x_pred + &k * (&z - &h_pad * &x_pred);
                                let p_next = (DMatrix::identity(3, 3) - &k * &h_pad) * &p_pred * (DMatrix::identity(3, 3) - &k * &h_pad).transpose() + &k * &c * &k.transpose(); 

                                self.x_est = x_next[(0, 0)];
                                self.y_est = x_next[(1, 0)];
                                self.theta_est = x_next[(2, 0)];
                            }
                            
                        }
                    }
                }
            }
            
           
            

        } else {
            // noisy movement with ideal speed actuation
            let speed = (self.dx.powi(2) + self.dy.powi(2)).sqrt();

            self.x_est += (self.dx / speed) * SPEED;
            self.y_est += (self.dy / speed) * SPEED;

            // estimated angle with ideal speed actuation
            self.theta_est = self.dy.atan2(self.dx);
        }

        

        // actual movement with noise in speed actuation

        let dx_samp = normal_dist_speed.sample(&mut thread_rng());
        let dy_samp = normal_dist_speed.sample(&mut thread_rng());

        let speed_actual = ((self.dx + dx_samp).powi(2) + (self.dy + dy_samp).powi(2)).sqrt();

        self.dx = ((self.dx + dx_samp) / speed_actual) * SPEED;
        self.dy = ((self.dy + dy_samp) / speed_actual) * SPEED;

        self.theta = (self.dy + dy_samp).atan2(self.dx + dx_samp);

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
            
            for point in original.iter_mut() {
                let xp = point.x - cx;
                let yp = point.y - cy;
                let xpp = xp * self.theta.cos() - yp * self.theta.sin();
                let ypp = xp * self.theta.sin() + yp * self.theta.cos();
                res.push(Vec2{x: xpp + cx, y: ypp + cy});
            }

            draw_triangle(res[0], res[1], res[2], self.color);
    }

    fn trilateration(&mut self,
                     anchors: &Vec<&Obstacle>,
                     anchor_measurements: &DMatrix<f32>, 
                     H: &mut DMatrix<f32>, 
                     Z: &mut DMatrix<f32>, 
                     C: &mut DMatrix<f32>, 
                     i: &usize, 
                     l: &usize,
                     number_anchors: &usize) {
                        for j in 0..number_anchors-1 {
                            H[(l + j, 0)] = 2.0 * (anchors[j+1].x - anchors[j].x);
                            H[(l + j, 1)] = 2.0 * (anchors[j+1].y - anchors[j].y);
                            Z[l + j] = -(anchor_measurements[(*i, j+1)]).powi(2) + (anchor_measurements[(*i, j)]).powi(2) 
                                        + anchors[j+1].x.powi(2) - anchors[j].x.powi(2) + anchors[j+1].y.powi(2) - anchors[j].y.powi(2);
                            
                            if j == 0 {
                                C[(l + j, l + j)] = 4.0 * ANCHOR_STD.powi(2) * (anchor_measurements[(*i, j+1)].powi(2) + anchor_measurements[(*i, j)].powi(2));
                                if *number_anchors > 2 {
                                    C[(l + j, l + j + 1)] = -4.0 * ANCHOR_STD.powi(2) * anchor_measurements[(*i, j+1)].powi(2);
                                }
                            } else if j < *number_anchors - 2 {
                                C[(l + j, l + j - 1)] =  -4.0 * ANCHOR_STD.powi(2) * anchor_measurements[(*i, j)].powi(2);
                                C[(l + j, l + j)] = 4.0 * ANCHOR_STD.powi(2) * (anchor_measurements[(*i, j+1)].powi(2) + anchor_measurements[(*i, j)].powi(2));
                                C[(l + j, l + j + 1)] =  -4.0 * ANCHOR_STD.powi(2) * anchor_measurements[(*i, j+1)].powi(2);
                            } else {
                                C[(l + j, l + j - 1)] =  -4.0 * ANCHOR_STD.powi(2) * anchor_measurements[(*i, j)].powi(2);
                                C[(l + j, l + j)] = 4.0 * ANCHOR_STD.powi(2) * (anchor_measurements[(*i, j+1)].powi(2) + anchor_measurements[(*i, j)].powi(2));
                            }

                        }
                     }

    fn dynamic(&mut self, omega: f32) -> DMatrix<f32> {
        let mut state= DMatrix::zeros(3, 1);
        state[(0, 0)] = self.x_est + SPEED * self.theta_est.cos() * MINIMUM_FRAME_TIME;
        state[(1, 0)] = self.y_est + SPEED * self.theta_est.sin() * MINIMUM_FRAME_TIME;
        state[(2, 0)] = self.theta_est + omega * MINIMUM_FRAME_TIME;
        state
    }

    
}

#[macroquad::main("MyGame")]
async fn main(){

    set_window_size(CANVAS_WIDTH, CANVAS_HEIGHT);

    let mut boids = (0..NUMBER_BOIDS).into_iter().map(|_| Boid::create()).collect::<Vec<_>>();
    let obstacles = (0..NUMBER_OBSTACLES).into_iter().map(|_| Obstacle::create()).collect::<Vec<_>>();
    let normal_dist_speed= Normal::new(0.0, SPEED_STD).unwrap();
    let normal_dist_anchors= Normal::new(0.0, ANCHOR_STD).unwrap();
    let mut escape = Escape::create(&obstacles);

    let mut prev_pos_segments : HashMap<Segment, Vec<BoidPosVel>> = HashMap::new();
    let mut obstacle_segments: HashMap<Segment, Vec<&Obstacle>> = HashMap::new();

    for obs in obstacles.iter() {
        obstacle_segments.entry(Segment{x: (obs.x as u32).div(SEGMENT_WIDTH), y: (obs.y as u32).div(SEGMENT_WIDTH)})
            .or_insert_with(Vec::new)
            .push(obs);
    }


    for boid in boids.iter() {
        prev_pos_segments.entry(Segment{x: (boid.x as u32).div(SEGMENT_WIDTH), y: (boid.y as u32).div(SEGMENT_WIDTH)})
            .or_insert_with(Vec::new)
            .push(BoidPosVel{x: boid.x, y: boid.y, dx: boid.dx, dy: boid.dy, color: boid.color});
    }

    let begin_time = SystemTime::now();
    let mut last_time = begin_time;

    let mut file = OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(true)
        .open("visualizer/data.csv")
        .expect("Unable to open file");

    writeln!(file, "Timestamp;Data")
        .expect("Something went wrong.");

    loop {
        // logging
        let current_time = begin_time.elapsed().unwrap().as_secs_f32() - last_time.elapsed().unwrap().as_secs_f32();
        last_time = SystemTime::now();

        let mut contents = String::new();
        contents += format!("{};[", current_time).as_str();

        let mut differences = Vec::new();

        // updating & drawing

        clear_background(Color { r: 180./255., g: 231./255., b: 237./255., a: 1.0 });

        let mut pos_segments : HashMap<Segment, Vec<BoidPosVel>> = HashMap::new();
        
        for i in 0..boids.len() {
            pos_segments.entry(boids.get_mut(i).unwrap().update(&prev_pos_segments,&obstacle_segments, &escape, &normal_dist_speed, &normal_dist_anchors))
                .or_insert_with(Vec::new)
                .push(BoidPosVel{x: boids.get_mut(i).unwrap().x, y: boids.get_mut(i).unwrap().y, dx: boids.get_mut(i).unwrap().dx, dy: boids.get_mut(i).unwrap().dy, color: boids.get_mut(i).unwrap().color});
            if boids.get_mut(i).unwrap().reached {
                
                continue;
            }
            boids.get_mut(i).unwrap().draw();

            // log position
            differences.push(boids.get_mut(i).unwrap().x - boids.get_mut(i).unwrap().x_est);
            differences.push(boids.get_mut(i).unwrap().y - boids.get_mut(i).unwrap().y_est);
            
        }

        for obs in obstacles.iter() {
            obs.draw();
        }

        escape.draw();

        prev_pos_segments = pos_segments;

        // logging
        if differences.len() > 0 {
            for i in (0..differences.len()).step_by(2) {
                contents += format!("[{}, {}],", differences[i], differences[i+1]).as_str();
            }
            contents.pop();
            contents += "]";

            writeln!(file, "{}", contents)
                .expect("Something went wrong");
        }
        


        let frame_time = get_frame_time();
        if frame_time < MINIMUM_FRAME_TIME {
            let time_to_sleep = (MINIMUM_FRAME_TIME - frame_time) * 1000.;
            std::thread::sleep(std::time::Duration::from_millis(time_to_sleep as u64));
        }
        next_frame().await
    }
    
}
