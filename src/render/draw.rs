use crate::model::world::World;
use macroquad::prelude::*;

pub fn draw_world(world: &World) {
    clear_background(BLACK);

    use_centered_camera();
    //balls
    for b in &world.balls {
        draw_circle(b.pos.0, b.pos.1, b.radius, RED);
    }
    // boundary
    draw_circle_lines(0.0, 0.0, world.boundary_radius, 2.0, WHITE);
}

fn use_centered_camera() {
    let (w, h) = (screen_width(), screen_height());
    set_camera(&Camera2D {
        target: vec2(0.0, 0.0),
        zoom: vec2(2.0 / w, -2.0 / h),
        ..Default::default()
    });
}
