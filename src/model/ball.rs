#[derive(Clone, Debug)]
pub struct Ball {
    pub pos: (f32, f32),
    pub vel: (f32, f32),
    pub radius: f32,
    pub hits: u32,
    pub hue: f32,
}
