//! Editor document: the in-memory scenario the CAD editor mutates, plus the
//! serde `Scenario` schema that mirrors the TOML files in `scenarios/`.
//!
//! `EditorDoc` is index-based (points are a flat `Vec<DVec3>`, elements
//! reference indices into it) so tools can weld/reuse coordinates cheaply.
//! `to_scenario`/`from_scenario` convert to/from the coordinate-based
//! `KiteDefinition` that kite-core actually consumes; kite-core re-welds
//! coincident coordinates within 1mm on build, so an index shared by two
//! elements here always maps to identical coordinates there — round-trip
//! stable by construction.

use std::collections::HashSet;

use glam::DVec3;
use kite_core::wind::{GustEvent, WindConfig};
use kite_core::{BridleLineDef, KiteDefinition, PanelDef, SparDef, StiffJunctionDef};
use serde::{Deserialize, Serialize};

/// 1mm weld tolerance, matching kite-core's `build_kite_from_def`.
const WELD_TOL: f64 = 1.0e-3;

// ---------------------------------------------------------------------
// TOML schema (scenarios/*.toml)
// ---------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Scenario {
    pub name: String,
    pub gravity: DVec3,
    pub duration: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wind: Option<WindConfig>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub kite: Option<KiteDefinition>,
    /// Additive optional `[sim]` table; omitted entirely when every override
    /// is unset so existing scenario files round-trip byte-for-byte in shape.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sim: Option<SimOverrides>,
}

/// Path-traversal guard for scenario filenames: non-empty, short, and
/// restricted to a safe character set so it can never resolve outside
/// `scenarios_dir` (no `.`, `/`, or `\`).
pub fn valid_name(name: &str) -> bool {
    !name.is_empty()
        && name.len() <= 64
        && name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
}

/// All-`Option` mirror of the `kite_core::world::Config` fields worth
/// overriding per-scenario. `None` = engine default.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct SimOverrides {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub substeps: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub iterations_per_substep: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub damping: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bladder_pressure: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub k_pressure: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ground_collision_enabled: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub self_collision_enabled: Option<bool>,
}

// ---------------------------------------------------------------------
// Editor-side element records (index-based)
// ---------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct SparItem {
    pub name: String,
    pub a: usize,
    pub b: usize,
    pub num_segments: usize,
    pub radius: f64,
    pub youngs_modulus: f64,
    pub shear_modulus: f64,
    pub density: f64,
}

#[derive(Debug, Clone)]
pub struct PanelItem {
    pub name: String,
    pub a: usize,
    pub b: usize,
    pub c: usize,
    pub warp_compliance: f64,
    pub weft_compliance: f64,
    pub shear_compliance: f64,
    pub bending_compliance: f64,
    pub subdivisions: usize,
    pub areal_density: f64,
}

#[derive(Debug, Clone)]
pub struct BridleItem {
    pub name: String,
    pub a: usize,
    pub b: usize,
    pub rest_length: f64,
    pub compliance: f64,
    pub diameter: f64,
    pub num_segments: usize,
    pub density: f64,
}

/// A bend-twist lock at a point index, freezing the as-built angle between
/// every spar segment touching it (masterplan's stiff-junction feature).
#[derive(Debug, Clone)]
pub struct StiffJointItem {
    pub point: usize,
    pub compliance: f64,
}

pub struct EditorDoc {
    pub name: String,
    pub gravity: DVec3,
    pub duration: f64,
    pub wind: WindConfig,
    pub sim: SimOverrides,
    pub points: Vec<DVec3>,
    pub spars: Vec<SparItem>,
    pub panels: Vec<PanelItem>,
    pub bridles: Vec<BridleItem>,
    pub junction: Option<usize>,
    pub junction_pinned: bool,
    pub pinned: HashSet<usize>,
    pub stiff_joints: Vec<StiffJointItem>,
}

impl Default for EditorDoc {
    fn default() -> Self {
        Self {
            name: "untitled".to_string(),
            gravity: DVec3::new(0.0, -9.81, 0.0),
            duration: 5.0,
            wind: WindConfig::default(),
            sim: SimOverrides::default(),
            points: Vec::new(),
            spars: Vec::new(),
            panels: Vec::new(),
            bridles: Vec::new(),
            junction: None,
            junction_pinned: false,
            pinned: HashSet::new(),
            stiff_joints: Vec::new(),
        }
    }
}

/// Finds an existing point within `WELD_TOL` of `p`, or appends a new one.
fn weld(points: &mut Vec<DVec3>, p: DVec3) -> usize {
    for (i, q) in points.iter().enumerate() {
        if (*q - p).length() < WELD_TOL {
            return i;
        }
    }
    points.push(p);
    points.len() - 1
}

impl EditorDoc {
    pub fn from_scenario(s: &Scenario) -> Self {
        let mut points = Vec::new();
        let mut spars = Vec::new();
        let mut panels = Vec::new();
        let mut bridles = Vec::new();
        let mut junction = None;
        let mut junction_pinned = false;
        let mut pinned = HashSet::new();
        let mut stiff_joints = Vec::new();

        if let Some(kite) = &s.kite {
            for sp in &kite.spars {
                let a = weld(&mut points, sp.start);
                let b = weld(&mut points, sp.end);
                spars.push(SparItem {
                    name: sp.name.clone(),
                    a,
                    b,
                    num_segments: sp.num_segments,
                    radius: sp.radius,
                    youngs_modulus: sp.youngs_modulus,
                    shear_modulus: sp.shear_modulus,
                    density: sp.density,
                });
            }
            for pn in &kite.panels {
                let a = weld(&mut points, pn.p1);
                let b = weld(&mut points, pn.p2);
                let c = weld(&mut points, pn.p3);
                panels.push(PanelItem {
                    name: pn.name.clone(),
                    a,
                    b,
                    c,
                    warp_compliance: pn.warp_compliance,
                    weft_compliance: pn.weft_compliance,
                    shear_compliance: pn.shear_compliance,
                    bending_compliance: pn.bending_compliance,
                    subdivisions: pn.subdivisions,
                    areal_density: pn.areal_density,
                });
            }
            junction = Some(weld(&mut points, kite.bridle_junction));
            junction_pinned = kite.bridle_junction_pinned;
            for br in &kite.bridles {
                let a = weld(&mut points, br.from);
                let b = weld(&mut points, br.to);
                bridles.push(BridleItem {
                    name: br.name.clone(),
                    a,
                    b,
                    rest_length: br.rest_length,
                    compliance: br.compliance,
                    diameter: br.diameter,
                    num_segments: br.num_segments,
                    density: br.density,
                });
            }
            for &pt in &kite.pinned_points {
                pinned.insert(weld(&mut points, pt));
            }
            for sj in &kite.stiff_junctions {
                stiff_joints.push(StiffJointItem {
                    point: weld(&mut points, sj.point),
                    compliance: sj.compliance,
                });
            }
        }

        Self {
            name: s.name.clone(),
            gravity: s.gravity,
            duration: s.duration,
            wind: s.wind.clone().unwrap_or_default(),
            sim: s.sim.clone().unwrap_or_default(),
            points,
            spars,
            panels,
            bridles,
            junction,
            junction_pinned,
            pinned,
            stiff_joints,
        }
    }

    pub fn to_scenario(&self) -> Scenario {
        let spars = self
            .spars
            .iter()
            .map(|sp| SparDef {
                name: sp.name.clone(),
                start: self.points[sp.a],
                end: self.points[sp.b],
                num_segments: sp.num_segments,
                radius: sp.radius,
                youngs_modulus: sp.youngs_modulus,
                shear_modulus: sp.shear_modulus,
                density: sp.density,
            })
            .collect();
        let panels = self
            .panels
            .iter()
            .map(|p| PanelDef {
                name: p.name.clone(),
                p1: self.points[p.a],
                p2: self.points[p.b],
                p3: self.points[p.c],
                warp_compliance: p.warp_compliance,
                weft_compliance: p.weft_compliance,
                shear_compliance: p.shear_compliance,
                bending_compliance: p.bending_compliance,
                subdivisions: p.subdivisions,
                areal_density: p.areal_density,
            })
            .collect();
        let bridles = self
            .bridles
            .iter()
            .map(|b| BridleLineDef {
                name: b.name.clone(),
                from: self.points[b.a],
                to: self.points[b.b],
                rest_length: b.rest_length,
                compliance: b.compliance,
                diameter: b.diameter,
                num_segments: b.num_segments,
                density: b.density,
            })
            .collect();
        let stiff_junctions = self
            .stiff_joints
            .iter()
            .map(|sj| StiffJunctionDef { point: self.points[sj.point], compliance: sj.compliance })
            .collect();
        let bridle_junction = self.junction.map(|i| self.points[i]).unwrap_or(DVec3::ZERO);
        let mut pinned_points: Vec<DVec3> = self.pinned.iter().map(|&i| self.points[i]).collect();
        // Stable order so save output doesn't jitter between hash-set iterations.
        pinned_points.sort_by(|a, b| a.to_array().partial_cmp(&b.to_array()).unwrap());

        // ponytail: kite name mirrors the scenario name; the editor has no
        // separate UI for it and nothing downstream reads it independently.
        let kite = Some(KiteDefinition {
            name: self.name.clone(),
            spars,
            panels,
            bridles,
            bridle_junction,
            bridle_junction_pinned: self.junction_pinned,
            pinned_points,
            stiff_junctions,
        });

        Scenario {
            name: self.name.clone(),
            gravity: self.gravity,
            duration: self.duration,
            wind: Some(self.wind.clone()),
            kite,
            sim: if self.sim == SimOverrides::default() {
                None
            } else {
                Some(self.sim.clone())
            },
        }
    }

    /// Weld-or-add a point, returning its index. Shared by the Add Point
    /// tool and any tool that needs to materialize a fresh coordinate.
    pub fn weld_point(&mut self, p: DVec3) -> usize {
        weld(&mut self.points, p)
    }

    /// Defaults cribbed from `scenarios/simple_kite_v1.toml`'s spar values.
    pub fn add_spar(&mut self, a: usize, b: usize) -> usize {
        let n = self.spars.len() + 1;
        self.spars.push(SparItem {
            name: format!("spar_{n}"),
            a,
            b,
            num_segments: 4,
            radius: 0.005,
            youngs_modulus: 4.0e10,
            shear_modulus: 4.0e9,
            density: 1950.0,
        });
        self.spars.len() - 1
    }

    /// Defaults cribbed from `scenarios/simple_kite_v1.toml`'s panel values.
    pub fn add_panel(&mut self, a: usize, b: usize, c: usize) -> usize {
        let n = self.panels.len() + 1;
        self.panels.push(PanelItem {
            name: format!("panel_{n}"),
            a,
            b,
            c,
            warp_compliance: 1.0e-4,
            weft_compliance: 1.0e-4,
            shear_compliance: 2.0e-4,
            bending_compliance: 0.1,
            subdivisions: 1,
            areal_density: 0.05,
        });
        self.panels.len() - 1
    }

    /// `rest_length` defaults to the current point distance.
    pub fn add_bridle(&mut self, a: usize, b: usize) -> usize {
        let n = self.bridles.len() + 1;
        let rest_length = (self.points[a] - self.points[b]).length();
        self.bridles.push(BridleItem {
            name: format!("bridle_{n}"),
            a,
            b,
            rest_length,
            compliance: 1.0e-6,
            diameter: 0.002,
            num_segments: 1,
            density: 970.0,
        });
        self.bridles.len() - 1
    }

    /// Toggles a stiff joint at point `idx`: adds one with the default
    /// compliance if absent, removes it if present.
    pub fn toggle_stiff_joint(&mut self, idx: usize) {
        match self.stiff_joints.iter().position(|sj| sj.point == idx) {
            Some(pos) => {
                self.stiff_joints.remove(pos);
            }
            None => self.stiff_joints.push(StiffJointItem { point: idx, compliance: 1.0e-9 }),
        }
    }

    /// Counts of spars/panels/bridles that reference `idx`, for the delete
    /// confirmation prompt.
    pub fn dependents_of_point(&self, idx: usize) -> (usize, usize, usize) {
        let spars = self.spars.iter().filter(|s| s.a == idx || s.b == idx).count();
        let panels = self
            .panels
            .iter()
            .filter(|p| p.a == idx || p.b == idx || p.c == idx)
            .count();
        let bridles = self.bridles.iter().filter(|b| b.a == idx || b.b == idx).count();
        (spars, panels, bridles)
    }

    /// Removes a point and every element that referenced it, then reindexes
    /// every remaining reference above `idx` down by one.
    pub fn delete_point(&mut self, idx: usize) {
        self.spars.retain(|s| s.a != idx && s.b != idx);
        self.panels.retain(|p| p.a != idx && p.b != idx && p.c != idx);
        self.bridles.retain(|b| b.a != idx && b.b != idx);
        if self.junction == Some(idx) {
            self.junction = None;
        }
        self.pinned.remove(&idx);
        self.stiff_joints.retain(|sj| sj.point != idx);
        self.points.remove(idx);

        let shift = |i: &mut usize| {
            if *i > idx {
                *i -= 1;
            }
        };
        for s in &mut self.spars {
            shift(&mut s.a);
            shift(&mut s.b);
        }
        for p in &mut self.panels {
            shift(&mut p.a);
            shift(&mut p.b);
            shift(&mut p.c);
        }
        for b in &mut self.bridles {
            shift(&mut b.a);
            shift(&mut b.b);
        }
        if let Some(j) = &mut self.junction {
            shift(j);
        }
        self.pinned = self.pinned.iter().map(|&i| if i > idx { i - 1 } else { i }).collect();
        for sj in &mut self.stiff_joints {
            shift(&mut sj.point);
        }
    }

    pub fn delete_spar(&mut self, idx: usize) {
        self.spars.remove(idx);
    }
    pub fn delete_panel(&mut self, idx: usize) {
        self.panels.remove(idx);
    }
    pub fn delete_bridle(&mut self, idx: usize) {
        self.bridles.remove(idx);
    }

    /// Adds a fresh gust with sensible defaults matching the current wind
    /// direction/strength.
    pub fn add_gust(&mut self) {
        let dir = if self.wind.direction.length_squared() > 1e-9 {
            self.wind.direction
        } else {
            DVec3::X
        };
        self.wind
            .gusts
            .push(GustEvent::new(0.0, 5.0, 2.0, dir, self.wind.v_ref.max(1.0)));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    fn load(name: &str) -> Scenario {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../scenarios")
            .join(name);
        let toml_str = std::fs::read_to_string(&path).expect("read scenario file");
        toml::from_str(&toml_str).expect("parse toml")
    }

    /// simple_kite_v1.toml -> EditorDoc -> Scenario must preserve the kite
    /// sub-tree and the pass-through top-level fields exactly (allowing for
    /// the doc's own added defaults, e.g. an absent `[sim]` table, and for
    /// `KiteDefinition::name` collapsing into the single `EditorDoc::name` —
    /// the editor has one name field, so a kite name that originally
    /// differed from the scenario name does not survive the round trip;
    /// that's an accepted, documented lossy spot, not a bug).
    #[test]
    fn roundtrip_simple_kite() {
        let scenario = load("simple_kite_v1.toml");
        let doc = EditorDoc::from_scenario(&scenario);
        let round = doc.to_scenario();

        assert_eq!(scenario.name, round.name);
        assert_eq!(scenario.gravity, round.gravity);
        assert_eq!(scenario.duration, round.duration);
        assert_eq!(
            serde_json::to_value(&scenario.wind).unwrap(),
            serde_json::to_value(&round.wind).unwrap()
        );

        let mut a = serde_json::to_value(&scenario.kite).unwrap();
        let mut b = serde_json::to_value(&round.kite).unwrap();
        for v in [&mut a, &mut b] {
            v.as_object_mut().unwrap().remove("name");
        }
        assert_eq!(a, b, "kite sub-tree roundtrip mismatch (ignoring kite name)");
        assert_eq!(round.kite.unwrap().name, round.name, "kite name follows scenario name");
        assert!(round.sim.is_none(), "no sim overrides were set, expect no [sim] table");
    }

    /// A doc with sim overrides and a pinned point survives a TOML
    /// string round trip (Scenario -> toml string -> Scenario -> EditorDoc).
    #[test]
    fn roundtrip_sim_overrides_and_pinned() {
        let scenario = load("simple_kite_v1.toml");
        let mut doc = EditorDoc::from_scenario(&scenario);
        doc.sim.substeps = Some(10);
        doc.sim.damping = Some(1.5);
        doc.pinned.insert(0);

        let toml_str = toml::to_string_pretty(&doc.to_scenario()).expect("serialize");
        let reparsed: Scenario = toml::from_str(&toml_str).expect("reparse");
        let doc2 = EditorDoc::from_scenario(&reparsed);

        assert_eq!(doc2.sim.substeps, Some(10));
        assert_eq!(doc2.sim.damping, Some(1.5));
        assert_eq!(doc2.pinned.len(), 1);
        let pinned_idx = *doc2.pinned.iter().next().unwrap();
        assert!((doc2.points[pinned_idx] - doc.points[0]).length() < WELD_TOL);
    }

    /// A doc with a subdivided panel, a segmented bridle, and a stiff joint
    /// survives a full TOML string round trip with every new field intact.
    #[test]
    fn roundtrip_subdivisions_segments_and_stiff_joint() {
        let mut doc = EditorDoc::default();
        let p0 = doc.weld_point(DVec3::new(0.0, 0.0, 0.0));
        let p1 = doc.weld_point(DVec3::new(1.0, 0.0, 0.0));
        let p2 = doc.weld_point(DVec3::new(0.0, 1.0, 0.0));
        let pi = doc.add_panel(p0, p1, p2);
        doc.panels[pi].subdivisions = 4;
        doc.panels[pi].areal_density = 0.08;
        let bi = doc.add_bridle(p0, p1);
        doc.bridles[bi].num_segments = 3;
        doc.bridles[bi].density = 1200.0;
        doc.toggle_stiff_joint(p0);
        doc.stiff_joints[0].compliance = 5.0e-8;

        let toml_str = toml::to_string_pretty(&doc.to_scenario()).expect("serialize");
        let reparsed: Scenario = toml::from_str(&toml_str).expect("reparse");
        let doc2 = EditorDoc::from_scenario(&reparsed);

        assert_eq!(doc2.panels[pi].subdivisions, 4);
        assert_eq!(doc2.panels[pi].areal_density, 0.08);
        assert_eq!(doc2.bridles[bi].num_segments, 3);
        assert_eq!(doc2.bridles[bi].density, 1200.0);
        assert_eq!(doc2.stiff_joints.len(), 1);
        assert_eq!(doc2.stiff_joints[0].compliance, 5.0e-8);
        assert!((doc2.points[doc2.stiff_joints[0].point] - doc.points[p0]).length() < WELD_TOL);
    }

    #[test]
    fn valid_name_rejects_traversal() {
        assert!(!valid_name("../evil"));
        assert!(!valid_name("a/b"));
        assert!(!valid_name(""));
        assert!(valid_name("simple_kite_v1"));
        assert!(!valid_name(&"x".repeat(65)));
    }
}
