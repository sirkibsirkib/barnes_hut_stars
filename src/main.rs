extern crate piston_window;

use piston_window::*;
extern crate rand;
use rand::{SeedableRng, Rng, Isaac64Rng};

macro_rules! sqr {
    ($x:expr) => {{
        $x * $x
    }}
}

const THRESH_DIST : f32 = 0.2;
const GRAV_CONSTANT : f32 = 0.000000001;

#[derive(Copy, Clone, Debug)]
struct Point {
    x : f32,
    y : f32,
}

impl Point {
    const NULL : Point = Point{x:0.0, y:0.0};
}

#[derive(Copy, Clone, Debug)]
struct Zone {
    tl : Point,
    width : f32,
}

impl Zone {
    fn contains(&self, p : &Point) -> bool {
        self.tl.x <= p.x && p.x < self.tl.x+self.width &&
        self.tl.y <= p.y && p.y < self.tl.y+self.width
    }

    fn split(&self) -> (Zone, Zone, Zone, Zone) {
        let w = self.width/2.0;
        (
            Zone {width : w, tl : self.tl},
            Zone {width : w, tl : Point{x:self.tl.x + w, y:self.tl.y}},
            Zone {width : w, tl : Point{x:self.tl.x, y:self.tl.y + w}},
            Zone {width : w, tl : Point{x:self.tl.x + w, y:self.tl.y + w}},
        )
    }
}

impl Point {
    fn dir_to(&self, other : &Point) -> f32 {
        if other.x < self.x {
            std::f32::consts::PI + ((other.y - self.y) / (other.x - self.x)).atan()
        } else {
            ((other.y - self.y) / (other.x - self.x)).atan()
        }
    }

    fn dist_to(&self, other : &Point) -> f32 {
        (sqr!(self.x - other.x) + sqr!(self.y - other.y)).sqrt()
    }

    #[inline]
    fn shift (&mut self, other : &Point) {
        self.x += other.x;
        self.y += other.y;
    }
}

#[derive(Debug,Clone)]
struct Body {
    p : Point,
    momentum : Point,
    mass : f32,
}

#[derive(Debug)]
enum MaybeNode {
    Nothing(Zone),
    One(Zone, Body),
    Something(Zone, Box<Node>),
}

#[derive(Debug)]
struct Node{
    virtual_body : Body,
    branches : [MaybeNode; 4],
}

fn rand_body<R : Rng>(r : &mut R) -> Body {
    Body {
        p : Point {
            x : r.gen::<f32>() * 0.2 + 0.4,
            y : r.gen::<f32>() * 0.2 + 0.4,
        },
        momentum : Point::NULL,
        mass : r.gen::<f32>() * 6.0 + 1.3,
    }
}

const WIDTH : f64 = 900.0;
const HEIGHT : f64 = 700.0;

fn main() {
    let mut r = Isaac64Rng::from_seed(&[54,43,45,5665]);
    let mut bodies : Vec<Body> = Vec::new();
    for _ in 0..0 {
        bodies.push(rand_body(&mut r));
    }
    let whole_zone = Zone {tl: Point::NULL, width:1.0};
    let mut last_mouse : Option<[f64 ; 2]> = None;
    let mut window: PistonWindow = WindowSettings::new("Hello Piston!", ((WIDTH) as u32, (HEIGHT) as u32))
        .exit_on_esc(true)
        .build()
        .unwrap_or_else(|e| { panic!("Failed to build PistonWindow: {}", e) });
    while let Some(e) = window.next() {
        if let Some(button) = e.release_args() {
            if button == Button::Mouse(MouseButton::Left) {
                if let Some(pos) = last_mouse {
                    let b = Body {
                        p : Point {
                            x : (pos[0] / WIDTH) as f32,
                            y : (pos[1] / HEIGHT) as f32,
                        },
                        momentum : Point::NULL,
                        mass : r.gen::<f32>() * 6.0 + 1.3,
                    };
                    bodies.push(b);
                    last_mouse = None;
                }
            }
        }

        if let Some(_) = e.mouse_relative_args() {
            continue;
        } else if let Some(z) = e.mouse_cursor_args() {
            last_mouse = Some(z);
        } else {
            window.draw_2d(&e, | _ , graphics| clear([0.0; 4], graphics));
            let tree = make_tree(whole_zone, bodies.to_vec());
            draw_quads(&e, &mut window, &tree, 0.06);
            draw_bodies(&bodies, &e, &mut window);
            for b in bodies.iter_mut() {
                apply_forces(b, &tree);
                b.p.shift(&b.momentum);
            }
            bodies.retain(|x| whole_zone.contains(&x.p));
        }
    }
}

fn draw_quads(event : &Event, window : &mut PistonWindow, n : &MaybeNode, opacity : f32) {
    match n {
        &MaybeNode::Nothing(_) => (),
        &MaybeNode::One(ref z, _) => {
            draw_rect(z, event, window, [1.0, 0.0, 0.0, opacity]);
        },
        &MaybeNode::Something(ref z, ref bn) => {
            draw_rect(z, event, window, [0.0, 0.0, 1.0, opacity]);
            draw_quads(event, window, &bn.branches[0], opacity);
            draw_quads(event, window, &bn.branches[1], opacity);
            draw_quads(event, window, &bn.branches[2], opacity);
            draw_quads(event, window, &bn.branches[3], opacity);
        },
    }
}

fn draw_rect(z : &Zone, event : &Event, window : &mut PistonWindow, col : [f32; 4]) {
    window.draw_2d
    (
        event, |context, graphics| {
                rectangle(col, [(z.tl.x as f64)*WIDTH, (z.tl.y as f64)*HEIGHT, (z.width) as f64 * WIDTH-1.0, (z.width) as f64 * HEIGHT-1.0],
                          context.transform,
                          graphics);
          }
    );
}

fn draw_bodies(bodies : &Vec<Body>, event : &Event, window : &mut PistonWindow) {
    for b in bodies {
        window.draw_2d(
            event, |context, graphics| {
                    rectangle([0.0, 1.0, 1.0, 1.0],
                              [(b.p.x as f64)*WIDTH, (b.p.y as f64)*HEIGHT, (b.mass*0.6) as f64, (b.mass*0.6) as f64],
                              context.transform,
                              graphics);
              }
        );
    }
}

fn apply_forces(body : &mut Body, n : &MaybeNode) {
    match n {
        &MaybeNode::Nothing(_) => (),
        &MaybeNode::One(_, ref b) => force(body, b),
        &MaybeNode::Something(_, ref x) => {
            if body.p.dist_to(& x.virtual_body.p) > THRESH_DIST {
                // using threshold
                force(body, &x.virtual_body)
            } else {
                let mn = & x.branches;
                for i in 0..4 {
                    apply_forces(body, &mn[i]);
                }
            };
        },
    }
}

fn force(pulled : &mut Body, other : &Body) {
    let dist = pulled.p.dist_to(& other.p);
    if dist <= 0.0 {
        return;
    }
    let dir = pulled.p.dir_to(& other.p);
    let power = GRAV_CONSTANT * other.mass / sqr!(dist + 0.01);
    let (dx, dy) = (power * (dir).cos(), power * (dir).sin());
    pulled.momentum.x += dx / pulled.mass;
    pulled.momentum.y += dy / pulled.mass;
}

fn make_tree(z : Zone, mut bodies : Vec<Body>) -> MaybeNode {
    if bodies.len() == 0 {
        MaybeNode::Nothing(z)
    } else if bodies.len() == 1 {
        MaybeNode::One(z, bodies.pop().unwrap())
    } else {
        let split_zone = z.split();

        let mut b0 : Vec<Body> = vec![];
        let mut b1 : Vec<Body> = vec![];
        let mut b2 : Vec<Body> = vec![];
        let mut b3 : Vec<Body> = vec![];
        let mut virtual_body = Body {
            p : Point::NULL,
            mass : 0.0,
            momentum : Point::NULL,
        };
        for b in bodies {
            let relevant = if split_zone.0.contains(&b.p){
                    &mut b0
                } else if split_zone.1.contains(&b.p){
                    &mut b1
                } else if split_zone.2.contains(&b.p){
                    &mut b2
                } else if split_zone.3.contains(&b.p){
                    &mut b3
                } else {
                panic!(format!("body {:?} not in {:?}", &b, &z));
            };
            virtual_body.p.x += b.p.x * b.mass;
            virtual_body.p.y += b.p.y * b.mass;
            virtual_body.mass += b.mass;
            relevant.push(b);
        }
        virtual_body.p.x /= virtual_body.mass;
        virtual_body.p.y /= virtual_body.mass;
        let n = Node {
            virtual_body : virtual_body,
            branches : [
                make_tree(split_zone.0, b0),
                make_tree(split_zone.1, b1),
                make_tree(split_zone.2, b2),
                make_tree(split_zone.3, b3),
            ]
        };
        MaybeNode::Something(z, Box::new(n))
    }
}
