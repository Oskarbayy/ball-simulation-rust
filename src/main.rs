use ball_sim_rust::model::world::World;
use ball_sim_rust::physics::physics;
use ball_sim_rust::render;
use ball_sim_rust::systems;

#[macroquad::main("Balls Sim")]
async fn main() {
    let mut world = World::new(320.0);
    systems::seed::reseed_world(&mut world, 1, 1);

    let mut acc = 0.0_f32;
    let dt_fixed = 1.0 / 120.0; // 120 Hz physics (pick 60/120)

    loop {
        let frame_dt = macroquad::prelude::get_frame_time();
        acc += frame_dt;

        // Run zero or more fixed-size physics steps to "catch up"
        while acc >= dt_fixed {
            physics::gravity_down(&mut world, 1000.0, dt_fixed);
            physics::integrate(&mut world, dt_fixed);
            physics::collide_balls(&mut world, 0.9);
            physics::collide_with_circle(&mut world);
            acc -= dt_fixed;
        }

        render::draw::draw_world(&world); // clears background
        macroquad::prelude::next_frame().await;
    }
}
