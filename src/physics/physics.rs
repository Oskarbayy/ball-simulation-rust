use crate::model::world::World;

pub fn integrate(world: &mut World, dt: f32) {
    for b in &mut world.balls {
        b.pos.0 += b.vel.0 * dt;
        b.pos.1 += b.vel.1 * dt;
    }
}

pub fn collide_with_circle(world: &mut World) {
    const EPS: f32 = 0.5; // small inward bias to avoid 1px bleed
    const E: f32 = 1.0; // restitution (1.0 = perfectly elastic)

    let r = world.boundary_radius;

    for b in &mut world.balls {
        let (x, y) = b.pos;
        let dist = (x * x + y * y).sqrt();

        // treat as collision a hair early so logic matches render
        if dist + b.radius > r - EPS {
            // outward unit normal
            let (nx, ny) = if dist != 0.0 {
                (x / dist, y / dist)
            } else {
                (1.0, 0.0)
            };

            // normal speed (positive when moving outward toward the wall)
            let vn = b.vel.0 * nx + b.vel.1 * ny;

            // REFLECT ONLY IF MOVING INTO THE WALL
            if vn > 0.0 {
                // v' = v - (1+e)*(v·n)*n  (with e=1 → flip the normal component)
                let k = (1.0 + E) * vn;
                b.vel.0 -= k * nx;
                b.vel.1 -= k * ny;
            }

            // snap just inside so we don't visually poke through next frame
            let target = (r - b.radius - EPS).max(0.0);
            b.pos.0 = nx * target;
            b.pos.1 = ny * target;

            b.hits = b.hits.saturating_add(1);
        }
    }
}

pub fn gravity_down(world: &mut World, g: f32, dt: f32) {
    for b in &mut world.balls {
        b.vel.1 -= g * dt;
    }
}

// might be too much copy pasta
pub fn collide_balls(world: &mut World, restitution: f32) {
    let n = world.balls.len();
    for i in 0..n {
        for j in (i + 1)..n {
            // get two distinct &mut Ball safely
            let (left, right) = world.balls.split_at_mut(j);
            let a = &mut left[i];
            let b = &mut right[0];

            // relative vector from a to b
            let dx = b.pos.0 - a.pos.0;
            let dy = b.pos.1 - a.pos.1;
            let dist2 = dx * dx + dy * dy;
            let rsum = a.radius + b.radius;

            // quick reject
            if dist2 >= rsum * rsum {
                continue;
            }

            // contact normal (unit)
            let dist = dist2.sqrt().max(1e-6);
            let nx = dx / dist;
            let ny = dy / dist;

            // --- 1) Positional correction (separate the overlap) ---
            let penetration = rsum - dist;
            let percent = 0.8; // push most of the overlap out
            let slop = 0.01; // ignore tiny overlaps
            let corr = (penetration - slop).max(0.0) * percent;
            a.pos.0 -= nx * corr * 0.5;
            a.pos.1 -= ny * corr * 0.5;
            b.pos.0 += nx * corr * 0.5;
            b.pos.1 += ny * corr * 0.5;

            // --- 2) Velocity impulse (elastic-ish bounce) ---
            let rvx = b.vel.0 - a.vel.0;
            let rvy = b.vel.1 - a.vel.1;
            let vel_n = rvx * nx + rvy * ny; // relative speed along normal
            if vel_n > 0.0 {
                continue;
            } // already separating

            // masses (choose a scheme); area ~ r^2 feels nice
            let ma = a.radius * a.radius;
            let mb = b.radius * b.radius;
            let inv_ma = 1.0 / ma;
            let inv_mb = 1.0 / mb;

            // impulse magnitude
            let j = -(1.0 + restitution) * vel_n / (inv_ma + inv_mb);
            let ix = j * nx;
            let iy = j * ny;

            // apply impulses
            a.vel.0 -= ix * inv_ma;
            a.vel.1 -= iy * inv_ma;
            b.vel.0 += ix * inv_mb;
            b.vel.1 += iy * inv_mb;

            a.hits = a.hits.saturating_add(1);
            b.hits = b.hits.saturating_add(1);
        }
    }
}

// pub fn gravity_towards_center(world: &mut World, k: f32, dt: f32) {
//     for b in &mut world.balls {
//         let (x, y) = b.pos;
//         let dist2 = x*x + y*y;
//         if dist2 > 0.0 {
//             // unit vector toward center is -pos/|pos|
//             let inv_len = 1.0 / dist2.sqrt();
//             let ax = -x * inv_len * k; // acceleration components
//             let ay = -y * inv_len * k;
//             b.vel.0 += ax * dt;
//             b.vel.1 += ay * dt;
//         }
//     }
// }
