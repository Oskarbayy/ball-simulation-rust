use crate::model::ball::Ball;
use crate::model::world::World;
use rand::rngs::SmallRng;
use rand::{Rng, SeedableRng};

/// Deterministically create `count` balls inside a circle of radius `boundary_r`.
fn randomerate_balls(seed: u64, count: usize, boundary_r: f32) -> Vec<Ball> {
    let mut rng = SmallRng::seed_from_u64(seed);
    let mut out = Vec::with_capacity(count);

    for _ in 0..count {
        // radius
        let radius = rng.random_range(6.0_f32..14.0);

        // position: pick a random angle and radius (bias-correct: sqrt on radius)
        let ang = rng.random_range(0.0_f32..std::f32::consts::TAU);
        let max_r = (boundary_r - 2.0 * radius).max(0.0);
        let dist = max_r * rng.random::<f32>().sqrt();
        let px = ang.cos() * dist;
        let py = ang.sin() * dist;

        // velocity: random direction * speed
        let dir_ang = rng.random_range(0.0_f32..std::f32::consts::TAU);
        let speed = rng.random_range(120.0_f32..220.0);
        let vx = dir_ang.cos() * speed;
        let vy = dir_ang.sin() * speed;

        // color as logical hue (0..1)
        let hue = rng.random::<f32>();

        out.push(Ball {
            pos: (px, py),
            vel: (vx, vy),
            radius,
            hits: 0,
            hue,
        });
    }

    out
}

pub fn reseed_world(world: &mut World, seed: u64, count: usize) {
    world.balls = randomerate_balls(seed, count, world.boundary_radius);
}
