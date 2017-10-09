extern crate piston_window;

use piston_window::*;
extern crate rand;
use rand::{SeedableRng, Rng, Isaac64Rng};

#[macro_use]
mod data;

use data::{Zone, Point, Body, Node, MaybeNode};

const WIDTH : f64 = 700.0;
const HEIGHT : f64 = 700.0;

const THRESH_DIST : f32 = 0.2;
const GRAV_CONSTANT : f32 = 0.000000004;

fn main() {
    let mut r = Isaac64Rng::from_seed(&[54,43,45,5665]);
    let mut bodies : Vec<Body> = Vec::new();
    let whole_zone = Zone {tl: Point::NULL, width:1.0};
    let mut tree : MaybeNode = MaybeNode::Nothing(whole_zone);

    let mut last_mouse : Option<[f64 ; 2]> = None;
    let mut window: PistonWindow = WindowSettings::new("N-Body!", ((WIDTH) as u32, (HEIGHT) as u32))
        .exit_on_esc(true)
        .build()
        .unwrap_or_else(|e| { panic!("Failed to build PistonWindow: {}", e) });


    let event_settings = EventSettings {
        max_fps: 32,
        ups: 64,
        ups_reset: 2,
        swap_buffers: true,
        bench_mode: false,
        lazy: false,
    };
    window.set_event_settings(event_settings);
    while let Some(e) = window.next() {
        if let Some(button) = e.release_args() {
            if button == Button::Mouse(MouseButton::Left) {
                if let Some(pos) = last_mouse {
                    let b = Body::new(
                        Point {
                            x : (pos[0] / WIDTH) as f32,
                            y : (pos[1] / HEIGHT) as f32,
                        },
                        r.gen::<f32>() * 9.0 + 1.0,
                    );
                    bodies.push(b);
                    last_mouse = None;
                }
            }
        }

        if let Some(_) = e.update_args() {
            tree = make_tree(whole_zone, bodies.to_vec());
            // println!("UPDATE");
            for b in bodies.iter_mut() {
                apply_forces(b, &tree);
                b.shift_with_momentum();
            }
            bodies = scrutinize(bodies,  &whole_zone);
        }

        if let Some(_) = e.render_args() {
            window.draw_2d(&e, | _ , graphics| clear([0.0; 4], graphics));
            draw_quads(&e, &mut window, &tree, 0.03);
            draw_bodies(&bodies, &e, &mut window);
        }



        if let Some(_) = e.mouse_relative_args() {
            continue;
        } else if let Some(z) = e.mouse_cursor_args() {
            last_mouse = Some(z);
        } else {

        }
    }
}

fn scrutinize(bodies : Vec<Body>, whole_zone : &Zone) -> Vec<Body> {
    let mut next : Vec<Body> = vec![];
    'outer : for b in bodies {
        if !whole_zone.contains(&b.p) {
            continue;
        }
        for n in next.iter_mut() {
            if b.dist_to(n) < 0.6*(n.radius() + b.radius()) {
                n.agglutinate(b);
                continue 'outer;
            }
        }
        next.push(b);
    }
    next
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
    window.draw_2d(event, |context, graphics| {
                rectangle(
                    col,
                    [
                        (z.tl.x as f64)*WIDTH,
                        (z.tl.y as f64)*HEIGHT,
                        (z.width) as f64 * WIDTH-1.0,
                        (z.width) as f64 * HEIGHT-1.0
                    ],
                    context.transform,
                    graphics,
                );
          }
    );
}

fn draw_bodies(bodies : &Vec<Body>, event : &Event, window : &mut PistonWindow) {
    for b in bodies {
        let h_rad = b.radius() as f64 * WIDTH;
        let v_rad = b.radius() as f64 * HEIGHT;
        window.draw_2d(event, |context, graphics| {
                    ellipse(
                        b.color(),
                        [
                            (b.p.x as f64)*WIDTH -h_rad,
                            (b.p.y as f64)*HEIGHT - v_rad,
                            h_rad*2.0,
                            v_rad*2.0
                        ],
                        context.transform,
                        graphics
                  );
              }
        );
    }
}

fn apply_forces(body : &mut Body, n : &MaybeNode) {
    match n {
        &MaybeNode::Nothing(_) => (),
        &MaybeNode::One(_, ref b) => force(body, b),
        &MaybeNode::Something(_, ref x) => {
            if body.dist_to(& x.virtual_body) > THRESH_DIST {
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
    let dist = pulled.dist_to(& other);
    if dist <= 0.0 {
        return;
    }
    let dir = pulled.dir_to(& other);
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
        let mut virtual_body = Body::new(Point::NULL, 0.0);
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
                    panic!();
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
