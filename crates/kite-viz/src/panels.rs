//! Left-panel inspector: element tree, selected-item editor, and the
//! global Scenario/Sim/Wind/View sections. Also the Simulate-mode inspector
//! (live cfg tinkering) and the bottom timeline/playback panel.

use eframe::egui;
use glam::DVec3;
use kite_core::wind::GustEvent;

use crate::app::ViewToggles;
use crate::doc::EditorDoc;
use crate::session::SimSession;
use crate::snapshot::Snapshot;
use crate::tools::{self, Selection, SelectionSet};
use crate::windviz::WindViz;

pub fn inspector(ui: &mut egui::Ui, doc: &mut EditorDoc, sel: &mut SelectionSet, view: &mut ViewToggles, windviz: &mut WindViz) {
    egui::ScrollArea::vertical().show(ui, |ui| {
        ui.heading("Inspector");
        selected_editor(ui, doc, sel);

        ui.separator();
        element_tree(ui, doc, sel);

        ui.separator();
        egui::CollapsingHeader::new("Scenario").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("Name");
                ui.text_edit_singleline(&mut doc.name);
            });
            ui.horizontal(|ui| {
                ui.label("Gravity");
                ui.add(egui::DragValue::new(&mut doc.gravity.x).speed(0.01).prefix("x: "));
                ui.add(egui::DragValue::new(&mut doc.gravity.y).speed(0.01).prefix("y: "));
                ui.add(egui::DragValue::new(&mut doc.gravity.z).speed(0.01).prefix("z: "));
            });
            ui.horizontal(|ui| {
                ui.label("Duration (s)");
                ui.add(egui::DragValue::new(&mut doc.duration).speed(0.1).range(0.0..=f64::MAX));
            });
        });

        egui::CollapsingHeader::new("Sim Config").show(ui, |ui| {
            opt_usize(ui, "Substeps", &mut doc.sim.substeps, 24);
            opt_usize(ui, "Iterations/substep", &mut doc.sim.iterations_per_substep, 1);
            opt_f64(ui, "Damping", &mut doc.sim.damping, 2.0, 0.01);
            opt_f64(ui, "Bladder pressure", &mut doc.sim.bladder_pressure, 0.0, 0.01);
            opt_f64(ui, "k_pressure", &mut doc.sim.k_pressure, 1.0, 0.01);
            opt_bool(ui, "Ground collision", &mut doc.sim.ground_collision_enabled, false);
            opt_bool(ui, "Self collision", &mut doc.sim.self_collision_enabled, false);
        });

        egui::CollapsingHeader::new("Wind").show(ui, |ui| {
            row_f64(ui, "v_ref", &mut doc.wind.v_ref, 0.1);
            row_f64(ui, "h_ref", &mut doc.wind.h_ref, 0.1);
            row_f64(ui, "shear_exponent", &mut doc.wind.shear_exponent, 0.01);
            ui.horizontal(|ui| {
                ui.label("direction");
                ui.add(egui::DragValue::new(&mut doc.wind.direction.x).speed(0.01).prefix("x: "));
                ui.add(egui::DragValue::new(&mut doc.wind.direction.y).speed(0.01).prefix("y: "));
                ui.add(egui::DragValue::new(&mut doc.wind.direction.z).speed(0.01).prefix("z: "));
            });
            row_f64(ui, "turbulence_intensity", &mut doc.wind.turbulence_intensity, 0.01);
            row_f64(ui, "length_scale", &mut doc.wind.length_scale, 0.5);
            ui.horizontal(|ui| {
                ui.label("octaves");
                ui.add(egui::DragValue::new(&mut doc.wind.octaves).range(0..=8));
            });
            ui.horizontal(|ui| {
                ui.label("seed");
                ui.add(egui::DragValue::new(&mut doc.wind.seed));
            });

            egui::CollapsingHeader::new(format!("Gusts ({})", doc.wind.gusts.len())).show(ui, |ui| {
                let mut remove = None;
                for (gi, g) in doc.wind.gusts.iter_mut().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(format!("#{gi}"));
                        ui.add(egui::DragValue::new(&mut g.start_time).speed(0.1).prefix("t0: "));
                        ui.add(egui::DragValue::new(&mut g.length).speed(0.1).prefix("len: "));
                        ui.add(egui::DragValue::new(&mut g.amplitude).speed(0.1).prefix("amp: "));
                        ui.add(egui::DragValue::new(&mut g.propagation_velocity).speed(0.1).prefix("v: "));
                        if ui.small_button("x").clicked() {
                            remove = Some(gi);
                        }
                    });
                }
                if let Some(gi) = remove {
                    doc.wind.gusts.remove(gi);
                }
                if ui.button("+ Add gust").clicked() {
                    doc.add_gust();
                }
            });
        });

        egui::CollapsingHeader::new("View").show(ui, |ui| {
            ui.checkbox(&mut view.draw_grid, "Draw grid");
            ui.checkbox(&mut view.draw_axes, "Draw axes");
        });

        windviz.inspector(ui);

        ui.separator();
        ui.small("Note: fabric mass hardcoded 20 g/vertex in importer.");
    });
}

fn row_f64(ui: &mut egui::Ui, label: &str, val: &mut f64, speed: f64) -> egui::Response {
    ui.horizontal(|ui| {
        ui.label(label);
        ui.add(egui::DragValue::new(val).speed(speed))
    })
    .inner
}

/// A checkbox that toggles the override on/off plus a `DragValue` for its
/// value, editable (including scientific notation via click-to-type) even
/// for very small compliances.
fn opt_f64(ui: &mut egui::Ui, label: &str, val: &mut Option<f64>, default: f64, speed: f64) {
    ui.horizontal(|ui| {
        let mut on = val.is_some();
        if ui.checkbox(&mut on, label).changed() {
            *val = if on { Some(default) } else { None };
        }
        let mut v = val.unwrap_or(default);
        if ui.add_enabled(on, egui::DragValue::new(&mut v).speed(speed)).changed() {
            *val = Some(v);
        }
    });
}

fn opt_usize(ui: &mut egui::Ui, label: &str, val: &mut Option<usize>, default: usize) {
    ui.horizontal(|ui| {
        let mut on = val.is_some();
        if ui.checkbox(&mut on, label).changed() {
            *val = if on { Some(default) } else { None };
        }
        let mut v = val.unwrap_or(default);
        if ui.add_enabled(on, egui::DragValue::new(&mut v).range(0..=10_000)).changed() {
            *val = Some(v);
        }
    });
}

fn opt_bool(ui: &mut egui::Ui, label: &str, val: &mut Option<bool>, default: bool) {
    ui.horizontal(|ui| {
        let mut on = val.is_some();
        if ui.checkbox(&mut on, format!("{label} (override)")).changed() {
            *val = if on { Some(default) } else { None };
        }
        let mut v = val.unwrap_or(default);
        if ui.add_enabled(on, egui::Checkbox::new(&mut v, "")).changed() {
            *val = Some(v);
        }
    });
}

/// Applies one row's click to the selection: shift toggles membership,
/// otherwise the click replaces the whole selection with just this element.
fn pick_row(ui: &egui::Ui, sel: &mut SelectionSet, picked: Selection) {
    if ui.input(|i| i.modifiers.shift) {
        sel.toggle(picked);
    } else {
        sel.set(picked);
    }
}

fn element_tree(ui: &mut egui::Ui, doc: &EditorDoc, sel: &mut SelectionSet) {
    egui::CollapsingHeader::new(format!("Points ({})", doc.points.len())).show(ui, |ui| {
        for (i, p) in doc.points.iter().enumerate() {
            let label = format!("point_{i}  [{:.2}, {:.2}, {:.2}]", p.x, p.y, p.z);
            if ui.selectable_label(sel.contains(Selection::Point(i)), label).clicked() {
                pick_row(ui, sel, Selection::Point(i));
            }
        }
    });
    egui::CollapsingHeader::new(format!("Spars ({})", doc.spars.len())).show(ui, |ui| {
        for (i, s) in doc.spars.iter().enumerate() {
            if ui.selectable_label(sel.contains(Selection::Spar(i)), &s.name).clicked() {
                pick_row(ui, sel, Selection::Spar(i));
            }
        }
    });
    egui::CollapsingHeader::new(format!("Panels ({})", doc.panels.len())).show(ui, |ui| {
        for (i, p) in doc.panels.iter().enumerate() {
            if ui.selectable_label(sel.contains(Selection::Panel(i)), &p.name).clicked() {
                pick_row(ui, sel, Selection::Panel(i));
            }
        }
    });
    egui::CollapsingHeader::new(format!("Bridles ({})", doc.bridles.len())).show(ui, |ui| {
        for (i, b) in doc.bridles.iter().enumerate() {
            if ui.selectable_label(sel.contains(Selection::Bridle(i)), &b.name).clicked() {
                pick_row(ui, sel, Selection::Bridle(i));
            }
        }
    });
}

fn selected_editor(ui: &mut egui::Ui, doc: &mut EditorDoc, sel: &mut SelectionSet) {
    if sel.len() > 1 {
        ui.heading(format!("{} selected", sel.len()));
        if ui.button("Delete").clicked() {
            let sels: Vec<Selection> = sel.iter().copied().collect();
            tools::delete_selected(doc, &sels);
            sel.clear();
        }
        return;
    }
    match sel.single().unwrap_or(Selection::None) {
        Selection::Point(i) if i < doc.points.len() => {
            ui.heading("Point");
            ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(&mut doc.points[i].x).speed(0.01).prefix("x: "));
                ui.add(egui::DragValue::new(&mut doc.points[i].y).speed(0.01).prefix("y: "));
                ui.add(egui::DragValue::new(&mut doc.points[i].z).speed(0.01).prefix("z: "));
            });
            let mut pinned = doc.pinned.contains(&i);
            if ui.checkbox(&mut pinned, "Pinned").changed() {
                if pinned {
                    doc.pinned.insert(i);
                } else {
                    doc.pinned.remove(&i);
                }
            }
            let mut is_junction = doc.junction == Some(i);
            if ui.checkbox(&mut is_junction, "Bridle junction").changed() {
                doc.junction = if is_junction { Some(i) } else { None };
            }
            if doc.junction == Some(i) {
                ui.checkbox(&mut doc.junction_pinned, "Junction pinned");
            }
            let mut stiff = doc.stiff_joints.iter().any(|sj| sj.point == i);
            if ui.checkbox(&mut stiff, "Stiff joint").changed() {
                doc.toggle_stiff_joint(i);
            }
            if let Some(sj) = doc.stiff_joints.iter_mut().find(|sj| sj.point == i) {
                row_f64(ui, "Joint compliance", &mut sj.compliance, 1.0e-10);
            }
            let mut lashed = doc.lashings.iter().any(|la| la.point == i);
            if ui.checkbox(&mut lashed, "Lashing").changed() {
                doc.toggle_lashing(i);
            }
            if let Some(la) = doc.lashings.iter_mut().find(|la| la.point == i) {
                row_f64(ui, "Lashing compliance", &mut la.compliance, 1.0e-10);
            }
        }
        Selection::Spar(i) if i < doc.spars.len() => {
            ui.heading("Spar");
            let s = &mut doc.spars[i];
            ui.horizontal(|ui| {
                ui.label("Name");
                ui.text_edit_singleline(&mut s.name);
            });
            row_usize(ui, "Segments", &mut s.num_segments, 1);
            row_f64(ui, "Radius (m)", &mut s.radius, 0.0005);
            row_f64(ui, "Young's modulus (Pa)", &mut s.youngs_modulus, 1.0e6);
            row_f64(ui, "Shear modulus (Pa)", &mut s.shear_modulus, 1.0e6);
            row_f64(ui, "Density (kg/m3)", &mut s.density, 1.0);
        }
        Selection::Panel(i) if i < doc.panels.len() => {
            ui.heading("Panel");
            let p = &mut doc.panels[i];
            ui.horizontal(|ui| {
                ui.label("Name");
                ui.text_edit_singleline(&mut p.name);
            });
            row_f64(ui, "Warp compliance", &mut p.warp_compliance, 1.0e-6);
            row_f64(ui, "Weft compliance", &mut p.weft_compliance, 1.0e-6);
            row_f64(ui, "Shear compliance", &mut p.shear_compliance, 1.0e-6);
            row_f64(ui, "Bending compliance", &mut p.bending_compliance, 1.0e-3);
            row_usize(ui, "Subdivisions", &mut p.subdivisions, 1);
            row_f64(ui, "Areal density (kg/m2)", &mut p.areal_density, 0.005);
        }
        Selection::Bridle(i) if i < doc.bridles.len() => {
            ui.heading("Bridle");
            let b = &mut doc.bridles[i];
            ui.horizontal(|ui| {
                ui.label("Name");
                ui.text_edit_singleline(&mut b.name);
            });
            row_f64(ui, "Rest length (m)", &mut b.rest_length, 0.01);
            row_f64(ui, "Compliance", &mut b.compliance, 1.0e-7);
            row_f64(ui, "Diameter (m)", &mut b.diameter, 0.0001);
            row_usize(ui, "Segments", &mut b.num_segments, 1);
            row_f64(ui, "Density (kg/m3)", &mut b.density, 10.0);
        }
        _ => {
            ui.label("(nothing selected)");
        }
    }
}

fn row_usize(ui: &mut egui::Ui, label: &str, val: &mut usize, min: usize) -> egui::Response {
    ui.horizontal(|ui| {
        ui.label(label);
        ui.add(egui::DragValue::new(val).range(min..=10_000))
    })
    .inner
}

/// Simulate-mode left panel. Per the live-tinkering invariant (see
/// `session.rs`), Sim Config/Wind/Gravity widgets here are bound DIRECTLY to
/// `session.world.cfg`; edits take effect on the next step, and are
/// mirrored back into `doc` afterwards so leaving/re-entering Simulate mode
/// keeps them. Structural fields (points/elements) are never touched here;
/// `is_stale` drives the Apply & Restart prompt for those.
pub fn simulate_inspector(
    ui: &mut egui::Ui,
    doc: &mut EditorDoc,
    session: &mut SimSession,
    view: &mut ViewToggles,
    windviz: &mut WindViz,
) {
    egui::ScrollArea::vertical().show(ui, |ui| {
        ui.heading("Inspector (Simulate)");

        if session.is_stale(doc) {
            ui.colored_label(
                egui::Color32::from_rgb(230, 180, 60),
                "Doc has structural edits since the live world was built.",
            );
            if ui.button("Apply & Restart").clicked() {
                session.rebuild(doc);
            }
            ui.separator();
        }

        egui::CollapsingHeader::new("Sim Config").default_open(true).show(ui, |ui| {
            let cfg = &mut session.world.cfg;
            if row_usize(ui, "Substeps", &mut cfg.substeps, 1).changed() {
                doc.sim.substeps = Some(cfg.substeps);
            }
            if row_usize(ui, "Iterations/substep", &mut cfg.iterations_per_substep, 1).changed() {
                doc.sim.iterations_per_substep = Some(cfg.iterations_per_substep);
            }
            if row_f64(ui, "Damping", &mut cfg.damping, 0.01).changed() {
                doc.sim.damping = Some(cfg.damping);
            }
            if row_f64(ui, "Bladder pressure", &mut cfg.bladder_pressure, 0.01).changed() {
                doc.sim.bladder_pressure = Some(cfg.bladder_pressure);
            }
            if row_f64(ui, "k_pressure", &mut cfg.k_pressure, 0.01).changed() {
                doc.sim.k_pressure = Some(cfg.k_pressure);
            }
            if ui.checkbox(&mut cfg.ground_collision_enabled, "Ground collision").changed() {
                doc.sim.ground_collision_enabled = Some(cfg.ground_collision_enabled);
            }
            if ui.checkbox(&mut cfg.self_collision_enabled, "Self collision").changed() {
                doc.sim.self_collision_enabled = Some(cfg.self_collision_enabled);
            }
            ui.horizontal(|ui| {
                ui.label("Gravity");
                ui.add(egui::DragValue::new(&mut cfg.gravity.x).speed(0.01).prefix("x: "));
                ui.add(egui::DragValue::new(&mut cfg.gravity.y).speed(0.01).prefix("y: "));
                ui.add(egui::DragValue::new(&mut cfg.gravity.z).speed(0.01).prefix("z: "));
            });
        });

        egui::CollapsingHeader::new("Wind").default_open(true).show(ui, |ui| {
            let wind = &mut session.world.cfg.wind;
            row_f64(ui, "v_ref", &mut wind.v_ref, 0.1);
            row_f64(ui, "h_ref", &mut wind.h_ref, 0.1);
            row_f64(ui, "shear_exponent", &mut wind.shear_exponent, 0.01);
            ui.horizontal(|ui| {
                ui.label("direction");
                ui.add(egui::DragValue::new(&mut wind.direction.x).speed(0.01).prefix("x: "));
                ui.add(egui::DragValue::new(&mut wind.direction.y).speed(0.01).prefix("y: "));
                ui.add(egui::DragValue::new(&mut wind.direction.z).speed(0.01).prefix("z: "));
            });
            row_f64(ui, "turbulence_intensity", &mut wind.turbulence_intensity, 0.01);
            row_f64(ui, "length_scale", &mut wind.length_scale, 0.5);
            ui.horizontal(|ui| {
                ui.label("octaves");
                ui.add(egui::DragValue::new(&mut wind.octaves).range(0..=8));
            });
            ui.horizontal(|ui| {
                ui.label("seed");
                ui.add(egui::DragValue::new(&mut wind.seed));
            });

            egui::CollapsingHeader::new(format!("Gusts ({})", wind.gusts.len())).show(ui, |ui| {
                let mut remove = None;
                for (gi, g) in wind.gusts.iter_mut().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(format!("#{gi}"));
                        ui.add(egui::DragValue::new(&mut g.start_time).speed(0.1).prefix("t0: "));
                        ui.add(egui::DragValue::new(&mut g.length).speed(0.1).prefix("len: "));
                        ui.add(egui::DragValue::new(&mut g.amplitude).speed(0.1).prefix("amp: "));
                        ui.add(egui::DragValue::new(&mut g.propagation_velocity).speed(0.1).prefix("v: "));
                        if ui.small_button("x").clicked() {
                            remove = Some(gi);
                        }
                    });
                }
                if let Some(gi) = remove {
                    wind.gusts.remove(gi);
                }
                if ui.button("+ Add gust").clicked() {
                    let dir = if wind.direction.length_squared() > 1e-9 { wind.direction } else { DVec3::X };
                    wind.gusts.push(GustEvent::new(0.0, 5.0, 2.0, dir, wind.v_ref.max(1.0)));
                }
            });
        });

        // Mirror gravity/wind back into doc every frame unconditionally.
        // Cheap (a handful of Copy fields plus one WindConfig clone) and
        // keeps them synced across a Simulate -> Edit -> Simulate round
        // trip. `doc.sim`'s Option fields are different: they distinguish
        // "explicit override" from "engine default" for scenario-file
        // output, so those are only written above, on actual edit, so a
        // field the user never touched stays `None` instead of getting
        // permanently baked into the saved TOML as an explicit value.
        doc.gravity = session.world.cfg.gravity;
        doc.wind = session.world.cfg.wind.clone();

        ui.separator();
        egui::CollapsingHeader::new("View").default_open(true).show(ui, |ui| {
            ui.checkbox(&mut view.draw_grid, "Draw grid");
            ui.checkbox(&mut view.draw_axes, "Draw axes");
            ui.checkbox(&mut view.show_aero, "Aero arrows");
            ui.checkbox(&mut view.show_cloth_force, "Cloth force overlay");
        });

        windviz.inspector(ui);
    });
}

/// Bottom timeline panel: playback transport, speed/dt controls, the history
/// scrubber with event ticks, and the energy sparkline.
pub fn timeline(ui: &mut egui::Ui, session: &mut SimSession, doc: &EditorDoc) {
    ui.horizontal(|ui| {
        if ui.button(if session.playing { "Pause" } else { "Play" }).clicked() {
            session.playing = !session.playing;
            if session.playing {
                session.scrub = None; // jump back to the live head
                session.budget = 0.0;
            }
        }
        if ui.button("Step").clicked() {
            session.playing = false;
            session.scrub = None;
            session.step_once();
        }
        if ui.button("Reset").clicked() {
            session.rebuild(doc);
        }
        ui.separator();
        ui.label("Speed");
        ui.add(egui::Slider::new(&mut session.speed, 0.25..=16.0).logarithmic(true));
        ui.label("dt");
        ui.add(egui::DragValue::new(&mut session.dt).speed(0.0005).range(0.0001..=0.1));
        ui.separator();
        let t_display = match session.scrub {
            Some(i) => session.history[i].t,
            None => session.world.time,
        };
        ui.label(format!("t = {t_display:.3}s"));
        ui.label(format!("rate = {:.2}x realtime", session.sim_rate));
        ui.separator();
        if ui.button("Bake").on_hover_text(format!("Runs {:.1}s synchronously", doc.duration)).clicked() {
            session.playing = false;
            session.scrub = None;
            session.bake(doc.duration);
        }
    });

    if let Some(t) = session.diverged {
        ui.colored_label(
            egui::Color32::RED,
            format!("DIVERGED at t = {t:.3}s — playback paused, history preserved for scrubbing"),
        );
    }

    let len = session.history.len();
    if len < 2 {
        return;
    }
    let mut idx = session.scrub.unwrap_or(len - 1);
    let label = if session.scrub.is_some() { "scrub" } else { "live" };
    let resp = ui.add(egui::Slider::new(&mut idx, 0..=len - 1).text(label));
    if resp.dragged() || resp.changed() {
        session.scrub = Some(idx);
        session.playing = false;
    }

    let frame_events = &session.history[idx].events;
    if !frame_events.is_empty() {
        ui.label(format!("Events this step: {frame_events:?}"));
    }

    let strip_rect = ui.available_rect_before_wrap();
    let (tick_rect, _) = ui.allocate_exact_size(egui::vec2(strip_rect.width(), 8.0), egui::Sense::hover());
    let painter = ui.painter_at(tick_rect);
    for m in &session.event_markers {
        let x = tick_rect.left() + (m.step as f32 / (len - 1) as f32) * tick_rect.width();
        let color = if m.broken { egui::Color32::RED } else { egui::Color32::YELLOW };
        painter.line_segment(
            [egui::pos2(x, tick_rect.top()), egui::pos2(x, tick_rect.bottom())],
            egui::Stroke::new(1.5, color),
        );
    }

    energy_sparkline(ui, &session.history);
}

/// Hand-drawn energy-over-history line plot (no `egui_plot` dependency).
fn energy_sparkline(ui: &mut egui::Ui, history: &[Snapshot]) {
    let width = ui.available_width();
    let (rect, _) = ui.allocate_exact_size(egui::vec2(width, 40.0), egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 0.0, egui::Color32::from_gray(20));

    let n = history.len();
    if n < 2 {
        return;
    }
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for s in history {
        lo = lo.min(s.energy);
        hi = hi.max(s.energy);
    }
    let span = (hi - lo).max(1e-9);
    let point = |i: usize| {
        let x = rect.left() + (i as f32 / (n - 1) as f32) * rect.width();
        let t = ((history[i].energy - lo) / span) as f32;
        egui::pos2(x, rect.bottom() - t * rect.height())
    };
    let stroke = egui::Stroke::new(1.0, egui::Color32::from_rgb(120, 200, 255));
    for i in 1..n {
        painter.line_segment([point(i - 1), point(i)], stroke);
    }
}
