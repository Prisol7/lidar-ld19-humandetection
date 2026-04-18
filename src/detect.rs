use std::fs::OpenOptions;
use std::io::Write;
use std::path::Path;

use kdtree::distance::squared_euclidean;
use kdtree::KdTree;
use serde::{Deserialize, Serialize};
use smartcore::ensemble::random_forest_classifier::RandomForestClassifier;
use smartcore::linalg::basic::matrix::DenseMatrix;

const DBSCAN_EPS: f64 = 0.10;
const DBSCAN_MIN_PTS: usize = 4;

const RULE_WIDTH_MIN: f64 = 0.30;
const RULE_WIDTH_MAX: f64 = 0.60;
const RULE_TORSO_RADIUS_MIN: f64 = 0.15;
const RULE_TORSO_RADIUS_MAX: f64 = 0.25;

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Features {
    pub width: f64,
    pub depth: f64,
    pub curvature: f64,
    pub point_count: f64,
}

#[derive(Debug, Clone)]
pub struct Cluster {
    pub points: Vec<(f64, f64)>,
    pub centroid: (f64, f64),
    pub features: Features,
    pub is_human: bool,
}

pub enum Classifier {
    Model(Box<RandomForestClassifier<f64, i64, DenseMatrix<f64>, Vec<i64>>>),
    Rule,
}

impl Classifier {
    pub fn load_or_fallback(model_path: &str) -> Self {
        if !Path::new(model_path).exists() {
            log::info!("No model at {model_path}, using rule-based classifier");
            return Classifier::Rule;
        }
        match std::fs::read(model_path) {
            Ok(bytes) => match bincode::deserialize::<
                RandomForestClassifier<f64, i64, DenseMatrix<f64>, Vec<i64>>,
            >(&bytes)
            {
                Ok(m) => {
                    log::info!("Loaded RandomForest model from {model_path}");
                    Classifier::Model(Box::new(m))
                }
                Err(e) => {
                    log::warn!("Failed to deserialize model: {e}. Falling back to rule");
                    Classifier::Rule
                }
            },
            Err(e) => {
                log::warn!("Failed to read {model_path}: {e}. Falling back to rule");
                Classifier::Rule
            }
        }
    }

    pub fn predict(&self, f: &Features) -> bool {
        match self {
            Classifier::Model(m) => {
                let row = vec![vec![f.width, f.depth, f.curvature, f.point_count]];
                let x = DenseMatrix::from_2d_vec(&row);
                m.predict(&x).map(|p| p[0] == 1).unwrap_or(false)
            }
            Classifier::Rule => rule_predict(f),
        }
    }
}

fn rule_predict(f: &Features) -> bool {
    let radius = if f.curvature > 1e-6 { 1.0 / f.curvature } else { f64::INFINITY };
    f.width >= RULE_WIDTH_MIN
        && f.width <= RULE_WIDTH_MAX
        && radius >= RULE_TORSO_RADIUS_MIN
        && radius <= RULE_TORSO_RADIUS_MAX
        && f.point_count >= DBSCAN_MIN_PTS as f64
}

pub fn cluster(points: &[(f64, f64)]) -> Vec<Vec<usize>> {
    let n = points.len();
    if n == 0 {
        return Vec::new();
    }

    let mut tree: KdTree<f64, usize, [f64; 2]> = KdTree::new(2);
    for (i, &(x, y)) in points.iter().enumerate() {
        if tree.add([x, y], i).is_err() {
            return Vec::new();
        }
    }

    let eps_sq = DBSCAN_EPS * DBSCAN_EPS;
    let mut labels: Vec<i32> = vec![-1; n];
    let mut visited = vec![false; n];
    let mut cluster_id: i32 = 0;

    for i in 0..n {
        if visited[i] {
            continue;
        }
        visited[i] = true;

        let neighbors = neighbors_of(&tree, points[i], eps_sq);
        if neighbors.len() < DBSCAN_MIN_PTS {
            continue;
        }

        labels[i] = cluster_id;
        let mut seeds = neighbors;
        let mut idx = 0;
        while idx < seeds.len() {
            let q = seeds[idx];
            if !visited[q] {
                visited[q] = true;
                let q_neighbors = neighbors_of(&tree, points[q], eps_sq);
                if q_neighbors.len() >= DBSCAN_MIN_PTS {
                    for nb in q_neighbors {
                        if !seeds.contains(&nb) {
                            seeds.push(nb);
                        }
                    }
                }
            }
            if labels[q] < 0 {
                labels[q] = cluster_id;
            }
            idx += 1;
        }
        cluster_id += 1;
    }

    let mut groups: Vec<Vec<usize>> = vec![Vec::new(); cluster_id as usize];
    for (i, &lab) in labels.iter().enumerate() {
        if lab >= 0 {
            groups[lab as usize].push(i);
        }
    }
    groups
}

fn neighbors_of(
    tree: &KdTree<f64, usize, [f64; 2]>,
    p: (f64, f64),
    eps_sq: f64,
) -> Vec<usize> {
    tree.within(&[p.0, p.1], eps_sq, &squared_euclidean)
        .map(|hits| hits.into_iter().map(|(_, &i)| i).collect())
        .unwrap_or_default()
}

pub fn extract_features(pts: &[(f64, f64)]) -> Features {
    let n = pts.len();
    if n < 2 {
        return Features { width: 0.0, depth: 0.0, curvature: 0.0, point_count: n as f64 };
    }

    let (mut a, mut b, mut width) = (0usize, 0usize, 0.0);
    for i in 0..n {
        for j in (i + 1)..n {
            let dx = pts[i].0 - pts[j].0;
            let dy = pts[i].1 - pts[j].1;
            let d2 = dx * dx + dy * dy;
            if d2 > width {
                width = d2;
                a = i;
                b = j;
            }
        }
    }
    let width = width.sqrt();

    let (ax, ay) = pts[a];
    let (bx, by) = pts[b];
    let chord_dx = bx - ax;
    let chord_dy = by - ay;
    let chord_len = (chord_dx * chord_dx + chord_dy * chord_dy).sqrt().max(1e-9);
    let mut depth = 0.0;
    for &(x, y) in pts {
        let num = (chord_dy * x - chord_dx * y + bx * ay - by * ax).abs();
        let dist = num / chord_len;
        if dist > depth {
            depth = dist;
        }
    }

    let curvature = fit_circle_curvature(pts);

    Features {
        width,
        depth,
        curvature,
        point_count: n as f64,
    }
}

fn fit_circle_curvature(pts: &[(f64, f64)]) -> f64 {
    let n = pts.len() as f64;
    if n < 3.0 {
        return 0.0;
    }
    let mx = pts.iter().map(|p| p.0).sum::<f64>() / n;
    let my = pts.iter().map(|p| p.1).sum::<f64>() / n;

    let (mut suu, mut suv, mut svv) = (0.0, 0.0, 0.0);
    let (mut suuu, mut svvv, mut suvv, mut svuu) = (0.0, 0.0, 0.0, 0.0);
    for &(x, y) in pts {
        let u = x - mx;
        let v = y - my;
        suu += u * u;
        suv += u * v;
        svv += v * v;
        suuu += u * u * u;
        svvv += v * v * v;
        suvv += u * v * v;
        svuu += v * u * u;
    }

    let det = suu * svv - suv * suv;
    if det.abs() < 1e-12 {
        return 0.0;
    }
    let rhs_u = 0.5 * (suuu + suvv);
    let rhs_v = 0.5 * (svvv + svuu);
    let uc = (rhs_u * svv - rhs_v * suv) / det;
    let vc = (suu * rhs_v - suv * rhs_u) / det;

    let r2 = uc * uc + vc * vc + (suu + svv) / n;
    if r2 <= 1e-9 {
        return 0.0;
    }
    1.0 / r2.sqrt()
}

pub fn analyze(foreground: &[(f64, f64)], classifier: &Classifier) -> Vec<Cluster> {
    let groups = cluster(foreground);
    let mut out = Vec::with_capacity(groups.len());
    for g in groups {
        let pts: Vec<(f64, f64)> = g.iter().map(|&i| foreground[i]).collect();
        let features = extract_features(&pts);
        let is_human = classifier.predict(&features);
        let cx = pts.iter().map(|p| p.0).sum::<f64>() / pts.len() as f64;
        let cy = pts.iter().map(|p| p.1).sum::<f64>() / pts.len() as f64;
        out.push(Cluster {
            points: pts,
            centroid: (cx, cy),
            features,
            is_human,
        });
    }
    out
}

pub fn log_features_csv(path: &str, clusters: &[Cluster]) {
    if clusters.is_empty() {
        return;
    }
    let exists = Path::new(path).exists();
    let mut file = match OpenOptions::new().create(true).append(true).open(path) {
        Ok(f) => f,
        Err(e) => {
            log::warn!("Cannot open {path}: {e}");
            return;
        }
    };
    if !exists {
        let _ = writeln!(file, "width,depth,curvature,point_count,label");
    }
    for c in clusters {
        let _ = writeln!(
            file,
            "{:.4},{:.4},{:.4},{},",
            c.features.width, c.features.depth, c.features.curvature, c.features.point_count as u32
        );
    }
}
