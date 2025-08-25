use crate::model::world::World;

pub fn integrate(world: &mut World, dt: f32) {
    for b in &mut world.balls {
        b.pos.0 += b.vel.0 * dt;
        b.pos.1 += b.vel.1 * dt;
    }
}

pub fn collide_with_circle(world: &mut World) {
    const EPS: f32 = 0.5; // small inward bias to avoid 1px poke-through
    let r = world.boundary_radius;

    for b in &mut world.balls {
        let (x, y) = b.pos;
        let dist = (x * x + y * y).sqrt();

        // treat as collision a hair early so logic == render
        if dist + b.radius >= r - EPS {
            // outward unit normal (handle center case)
            let (nx, ny) = if dist != 0.0 {
                (x / dist, y / dist)
            } else {
                (1.0, 0.0)
            };

            // reflect: v' = v - 2*(v·n)*n
            let dot = b.vel.0 * nx + b.vel.1 * ny;
            b.vel.0 -= 2.0 * dot * nx;
            b.vel.1 -= 2.0 * dot * ny;

            // snap exactly just-inside the wall to avoid visual bleed
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

// --- helpers: position and distance-to-boundary at time t (with gravity down) ---
#[inline]
fn pos_at(px: f32, py: f32, vx: f32, vy: f32, g: f32, t: f32) -> (f32, f32) {
    // x(t) = px + vx t
    // y(t) = py + vy t - 0.5 g t^2   (gravity pulls down)
    (px + vx * t, py + vy * t - 0.5 * g * t * t)
}

#[inline]
fn dist_minus_R(px: f32, py: f32, vx: f32, vy: f32, g: f32, t: f32, r_center: f32) -> f32 {
    let (x, y) = pos_at(px, py, vx, vy, g, t);
    (x * x + y * y).sqrt() - r_center
}

// Find a hit time in (0, dt] by sampling to bracket, then bisect.
// Returns Some(t_hit) if we cross the boundary.
fn predict_wall_hit(
    px: f32,
    py: f32,
    vx: f32,
    vy: f32,
    g: f32,
    r_center: f32,
    dt: f32,
) -> Option<f32> {
    // We are inside the circle normally: f(0) <= 0
    let mut t0 = 0.0;
    let mut f0 = dist_minus_R(px, py, vx, vy, g, t0, r_center);

    // Coarse sampling to find a bracket [t0, t1] where f crosses 0
    let steps = 8; // increase to 16 if g is huge
    for i in 1..=steps {
        let t1 = dt * (i as f32) / (steps as f32);
        let f1 = dist_minus_R(px, py, vx, vy, g, t1, r_center);
        if f0 <= 0.0 && f1 >= 0.0 {
            // bisection refine
            let mut lo = t0;
            let mut hi = t1;
            for _ in 0..16 {
                let mid = 0.5 * (lo + hi);
                let fm = dist_minus_R(px, py, vx, vy, g, mid, r_center);
                if fm >= 0.0 {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
            return Some(0.5 * (lo + hi));
        }
        t0 = t1;
        f0 = f1;
    }
    None
}

// Advance ONE ball by dt using gravity, committing wall bounces exactly.
// Uses remaining time after the hit, so it doesn't lose energy.
pub fn advance_ball_wall(b: &mut crate::model::ball::Ball, r_boundary: f32, g: f32, dt: f32) {
    // Work in center-radius space
    let r_center = (r_boundary - b.radius).max(0.0);

    // local state (so we can loop within dt)
    let mut px = b.pos.0;
    let mut py = b.pos.1;
    let mut vx = b.vel.0;
    let mut vy = b.vel.1;

    let mut rem = dt;

    while rem > 0.0 {
        if let Some(thit) = predict_wall_hit(px, py, vx, vy, g, r_center, rem) {
            // Advance to impact
            let (hx, hy) = pos_at(px, py, vx, vy, g, thit);
            let vxt = vx;
            let vyt = vy - g * thit; // v(t) = v0 + a t

            // Outward normal at hit (unit)
            let len = (hx * hx + hy * hy).sqrt().max(1e-6);
            let nx = hx / len;
            let ny = hy / len;

            // Reflect the normal component: v' = v - 2*(v·n)*n  (e = 1)
            let vn = vxt * nx + vyt * ny; // NOTE: for inner→outer hit, vn > 0
            let rvx = vxt - 2.0 * vn * nx;
            let rvy = vyt - 2.0 * vn * ny;

            // Commit at the hit
            px = hx;
            py = hy;
            vx = rvx;
            vy = rvy;

            // Use the remaining time of this mini-step
            let rem2 = rem - thit;
            px += vx * rem2;
            py += vy * rem2 - 0.5 * g * rem2 * rem2;
            vy -= g * rem2;

            rem = 0.0; // we consumed the whole dt chunk
        } else {
            // No hit in the remainder: advance normally and finish
            px += vx * rem;
            py += vy * rem - 0.5 * g * rem * rem;
            vy -= g * rem;
            rem = 0.0;
        }
    }

    b.pos.0 = px;
    b.pos.1 = py;
    b.vel.0 = vx;
    b.vel.1 = vy;
}

// Advance & commit the whole world by dt (wall only).
pub fn advance_and_commit(world: &mut World, dt: f32, g: f32) {
    let r = world.boundary_radius;
    for b in &mut world.balls {
        advance_ball_wall(b, r, g, dt);
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
