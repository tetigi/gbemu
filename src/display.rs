use sdl2::pixels::Color;
use sdl2::rect::Rect;
use sdl2::render::WindowCanvas;

pub enum Colour {
    White,
    LightGrey,
    DarkGrey,
    Black,
}

impl Into<Color> for Colour {
    fn into(self) -> Color {
        match self {
            Colour::White => Color::RGB(255, 255, 255),
            Colour::LightGrey => Color::RGB(170, 170, 170),
            Colour::DarkGrey => Color::RGB(85, 85, 85),
            Colour::Black => Color::RGB(0, 0, 0),
        }
    }
}

pub struct Display {
    canvas: WindowCanvas,
}

impl Display {
    pub fn init() -> Display {
        let sdl_context = sdl2::init().unwrap();
        let video_subsystem = sdl_context.video().unwrap();

        let window = video_subsystem
            .window("gbemu", 320, 288)
            .position_centered()
            .build()
            .unwrap();

        let mut canvas = window.into_canvas().build().unwrap();

        canvas.set_draw_color(Colour::White);
        canvas.clear();
        canvas.present();

        canvas.clear();
        canvas.present();
        Display { canvas }
    }

    pub fn write_pixel(&mut self, x: i32, y: i32, colour: Colour) {
        self.canvas.set_draw_color(colour);
        self.canvas
            .fill_rect(Rect::new(2 * x, 2 * y, 2, 2))
            .unwrap();
    }

    pub fn refresh(&mut self) {
        self.canvas.present();
    }
}
