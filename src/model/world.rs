use super::ball::Ball;

pub struct World {
    pub boundary_radius: f32,
    pub balls: Vec<Ball>,
}

impl World {
    pub fn new(boundary_radius: f32) -> Self {
        let mut balls = Vec::new();
        balls.push(Ball {
            pos: (50.0, 0.0),
            vel: (600.0, 150.0),
            radius: 10.0,
            hits: 0,
            hue: 0.0,
        });

        Self {
            boundary_radius,
            balls: balls,
        }
    }
}
