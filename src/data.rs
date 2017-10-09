use std;


macro_rules! sqr {
    ($x:expr) => {{
        $x * $x
    }}
}

#[derive(Debug,Clone)]
pub struct Body {
    pub p : Point,
    pub momentum : Point,
    pub mass : f32,
    radius : f32,
}

impl Body {
    pub fn dir_to(&self, other : &Body) -> f32 {
        self.p.dir_to(&other.p)
    }

    pub fn dist_to(&self, other : &Body) -> f32 {
        self.p.dist_to(&other.p)
    }

    #[inline]
    pub fn radius(&self) -> f32 {
        self.radius
    }

    pub fn shift_with_momentum(&mut self) {
        self.p.shift(&self.momentum);
    }

    pub fn new(p : Point, mass : f32) -> Body {
        Body {
            p : p,
            momentum : Point::NULL,
            mass : mass,
            radius : Self::radius_from_mass(mass),
        }
    }

    fn radius_from_mass(mass : f32) -> f32 {
        mass.powf(0.3333) * 0.001
    }

    pub fn color(&self) -> [f32 ; 4] {
        [0.0, 1.0, 0.0, 1.0]
    }

    pub fn agglutinate(&mut self, other : Body) {

        let new_mass = self.mass + other.mass;
        let new_p = Point {
            x : (self.p.x * self.mass + other.p.x * other.mass) / new_mass,
            y : (self.p.y * self.mass + other.p.y * other.mass) / new_mass,
        };
        let new_momentum = Point {
            x : (self.momentum.x * self.mass + other.momentum.x * other.mass) / new_mass,
            y : (self.momentum.y * self.mass + other.momentum.y * other.mass) / new_mass,
        };
        self.p = new_p;
        self.momentum = new_momentum;
        self.mass = new_mass;
        self.radius = Self::radius_from_mass(new_mass);
    }
}




#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Point {
    pub x : f32,
    pub y : f32,
}


impl Point {
    pub const NULL : Point = Point{x:0.0, y:0.0};
}

#[derive(Copy, Clone, Debug)]
pub struct Zone {
    pub tl : Point,
    pub width : f32,
}

impl Zone {
    pub fn contains(&self, p : &Point) -> bool {
        self.tl.x <= p.x && p.x < self.tl.x+self.width &&
        self.tl.y <= p.y && p.y < self.tl.y+self.width
    }

    pub fn split(&self) -> (Zone, Zone, Zone, Zone) {
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

#[derive(Debug)]
pub enum MaybeNode {
    Nothing(Zone),
    One(Zone, Body),
    Something(Zone, Box<Node>),
}

#[derive(Debug)]
pub struct Node{
    pub virtual_body : Body,
    pub branches : [MaybeNode; 4],
}
