extern crate piston_window;

use piston_window::*;
extern crate rand;
use rand::{SeedableRng, Rng, Isaac64Rng};

macro_rules! sqr {
    ($x:expr) => {{
        $x * $x
    }}
}

const THRESH_DIST : f32 = 0.3;
const GRAV_CONSTANT : f32 = 0.0000001;

#[derive(Copy, Clone, Debug)]
struct Point {
    x : f32,
    y : f32,
}

impl Point {
    fn avg(&self, p : &Point) -> Point {
        Point {
            x : (self.x + p.x) / 2.0,
            y : (self.y + p.y) / 2.0,
        }
    }

    fn dir_to(&self, other : &Point) -> f32 {
        ((other.y - self.y) / (other.x - self.x)).atan()
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
    mass : f32,
}

#[derive(Debug)]
enum MaybeNode {
    Nothing,
    One(Body),
    Something(Box<Node>),
}

impl MaybeNode {
    fn count_bodies(&self) -> usize {
        use MaybeNode::*;
        match self {
            &Nothing => 0,
            &Something(ref n) => n.count_bodies(),
            &One(_) => 1,
        }
    }

    fn depth(&self) -> u32 {
        match self {
            &MaybeNode::Nothing => 0,
            &MaybeNode::One(_) => 1,
            &MaybeNode::Something(ref b) => b.depth(),
        }
    }
}

#[derive(Debug)]
struct Node{
    virtual_body : Body,
    branches : [MaybeNode; 4],
}

impl Node {
    fn count_bodies(&self) -> usize {
        let mut total = 0;
        for m in self.branches.iter() {
            total += m.count_bodies();
        }
        total
    }
    fn depth(&self) -> u32 {
        let mut m = 0;
        for i in 0..4 {
            m = std::cmp::max(m, self.branches[0].depth());
        }
        m + 1
    }
}

fn rand_body<R : Rng>(r : &mut R) -> Body {
    Body {
        p : Point {
            x : r.gen::<f32>() * 0.8 + 0.1,
            y : r.gen::<f32>() * 0.8 + 0.1,
        },
        mass : r.gen::<f32>() * 10.0,
    }
}

const WIDTH : f64 = 640.0;
const HEIGHT : f64 = 480.0;

fn main() {
    let mut r = Isaac64Rng::from_seed(&[54,43,45,5665]);

    let mut bodies : Vec<Body> = Vec::new();
    for _ in 0..30 {
        bodies.push(rand_body(&mut r));
    }
    // println!("count bodies = {}", tree.count_bodies());

    // let depth = tree.depth();



    let sleep_time = std::time::Duration::from_millis(1000);
    let mut window: PistonWindow = WindowSettings::new("Hello Piston!", (640, 480))
        .exit_on_esc(true)
        .build()
        .unwrap_or_else(|e| { panic!("Failed to build PistonWindow: {}", e) });
    while let Some(e) = window.next() {
        while let Some(event) = window.next() {
            window.draw_2d(&event, | _ , graphics| clear([0.0; 4], graphics));
            draw_bodies(&bodies, event, &mut window);
            let tree = make_tree(bodies.to_vec(), Point{x:0.0, y:0.0}, Point{x:1.0, y:1.0});
            for b in bodies.iter_mut() {
                let mut impulse = Point {x:0.0, y:0.0};
                step_for(b, &mut impulse, &tree);
                b.p.shift(&impulse);
            }

            // std::thread::sleep(sleep_time);
            println!("STEP");
            // event == ();
            // window.draw_2d
            // (
            //     &event, |context, graphics| {
            //             // clear([1.0; 4], graphics);
            //             rectangle([1.0, 0.0, 0.0, 0.3], // red
            //                       [0.0, 0.0, WIDTH, HEIGHT],
            //                       context.transform,
            //                       graphics);
            //       }
            // );
        }
    }
}

fn draw_tree(n : &MaybeNode, tree_depth : u32, event : piston_window::Event, window : &mut piston_window::PistonWindow){
    unimplemented!();
}

fn draw_bodies(bodies : &Vec<Body>, event : piston_window::Event, window : &mut piston_window::PistonWindow) {
    for b in bodies {
        window.draw_2d
        (
            &event, |context, graphics| {
                    // clear([1.0; 4], graphics);
                    rectangle([1.0, 0.0, 1.0, 1.0], // red
                              [(b.p.x as f64)*WIDTH, (b.p.y as f64)*HEIGHT, 3.0, 3.0],
                              context.transform,
                              graphics);
              }
        );
    }
}

fn step_for(body : &mut Body, impulse : &mut Point, n : &MaybeNode) {
    match n {
        &MaybeNode::Nothing => (),
        &MaybeNode::One(ref b) => force(body, impulse, b),
        &MaybeNode::Something(ref x) => {
            if body.p.dist_to(& x.virtual_body.p) <= THRESH_DIST {
                force(body, impulse, &x.virtual_body)
            } else {
                let mn = & x.branches;
                step_for(body, impulse, &mn[0]);
                step_for(body, impulse, &mn[1]);
                step_for(body, impulse, &mn[2]);
                step_for(body, impulse, &mn[3]);
            };
        },
    }
}

fn same_object<T>(a: &T, b: &T) -> bool {
    a as *const T == b as *const T
}

fn force(pulled : &mut Body, impulse : &mut Point, other : &Body) {
    let dist = pulled.p.dist_to(& other.p);
    if dist == 0.0 {
        return;
    }
    let dir = pulled.p.dir_to(& other.p);
    let power = GRAV_CONSTANT * other.mass / sqr!(dist);
    impulse.x += power * (dir).cos();
    impulse.y += power * (dir).sin();
}

fn make_tree(mut bodies : Vec<Body>, tl : Point, br : Point) -> MaybeNode {
    if bodies.len() == 0 {
        MaybeNode::Nothing
    } else if bodies.len() == 1 {
        MaybeNode::One(bodies.pop().unwrap())
    } else {
        let (avgx, avgy) = ((tl.x + br.x)/2.0, (tl.y + br.y)/2.0);
        let avg = Point{x:avgx, y:avgy};
        let mut b00 : Vec<Body> = vec![];
        let mut b01 : Vec<Body> = vec![];
        let mut b10 : Vec<Body> = vec![];
        let mut b11 : Vec<Body> = vec![];
        let mut virtual_body = Body {p : Point{x:0.0, y:0.0}, mass:0.0};
        for b in bodies {
            let relevant = if b.p.x < avgx {
                if b.p.y < avgy {&mut b00} else {&mut b10}
            } else {
                if b.p.y < avgy {&mut b01} else {&mut b11}
            };
            if b.p.x < tl.x || b.p.x > br.x || b.p.y < tl.y || b.p.y > br.y {
                panic!(format!("body {:?} not inbetween tl {:?} and br {:?}", b.p, tl, br));
            }
            virtual_body.p.x += b.p.x;
            virtual_body.p.y += b.p.y;
            virtual_body.mass += b.mass;
            relevant.push(b);
        }
        virtual_body.p.x /= virtual_body.mass;
        virtual_body.p.y /= virtual_body.mass;
        let n = Node {
            virtual_body : virtual_body,
            branches : [
                make_tree(b00, tl, avg),
                make_tree(b01, Point{x:avgx, y:tl.y}, Point{x:br.x, y:avgy}),
                make_tree(b10, Point{x:tl.x, y:avgy}, Point{x:avgx, y:br.y}),
                make_tree(b11, avg, br),
            ]
        };
        MaybeNode::Something(Box::new(n))
    }
}
