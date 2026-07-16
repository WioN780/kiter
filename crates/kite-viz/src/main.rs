mod app;
mod doc;
mod panels;
mod render;
mod session;
mod snapshot;
mod tools;
mod viewport;
mod windviz;

use app::App;

fn main() -> eframe::Result<()> {
    let native_options = eframe::NativeOptions {
        renderer: eframe::Renderer::Wgpu,
        depth_buffer: 24,
        viewport: eframe::egui::ViewportBuilder::default().with_inner_size([1500.0, 950.0]),
        ..Default::default()
    };
    eframe::run_native(
        "kite-viz",
        native_options,
        Box::new(|cc| Ok(Box::new(App::new(cc)))),
    )
}
