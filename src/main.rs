use ball_sim_rust::model::world::World;
use ball_sim_rust::physics::physics;
use ball_sim_rust::render;
use ball_sim_rust::systems;

#[macroquad::main("Balls Sim")]
async fn main() {
    let mut world = World::new(320.0);
    systems::seed::reseed_world(&mut world, 1, 20);

    let mut acc = 0.0_f32;
    let dt_fixed = 1.0 / 120.0; // 120 Hz physics (pick 60/120)

    loop {
        let frame_dt = macroquad::prelude::get_frame_time();
        acc += frame_dt;

        // Run zero or more fixed-size physics steps to "catch up"
        while acc >= dt_fixed {
            let sub = 2; // you can keep substeps low now
            let h = dt_fixed / sub as f32;
            for _ in 0..sub {
                physics::advance_and_commit(&mut world, h, 1000.0);
                physics::collide_balls(&mut world, 0.9); // ball–ball stays as-is
            }
            physics::collide_balls(&mut world, 0.9);
            acc -= dt_fixed;
        }

        render::draw::draw_world(&world); // clears background
        macroquad::prelude::next_frame().await;
    }
}
