use lidar_ld19::{LD19, DIR_ROUND};
use minifb::{Key, Window, WindowOptions};
use std::time::{Duration, Instant};

const PORT: &str = "/dev/ttyUSB0";
const MAX_RANGE_M: f64 = 4.0;
const WIDTH: usize = 800;
const HEIGHT: usize = 800;

const CALIBRATION: Duration = Duration::from_secs(30);
const ANGLE_BINS: usize = 360;
const MATCH_TOLERANCE_M: f64 = 0.15;
const MIN_BG_SAMPLES: u32 = 5;

const BG_COLOR: u32 = 0x00_0a_0a_0a;
const GRID: u32 = 0x00_1a_2a_1a;
const STATIC_COLOR: u32 = 0x00_ff_ff_ff;
const MOTION_COLOR: u32 = 0x00_ff_30_30;
const ORIGIN: u32 = 0x00_ff_ff_00;
const CAL_COLOR: u32 = 0x00_00_c8_ff;

enum Phase {
    Calibrating { started: Instant, samples: Vec<Vec<f64>> },
    Detecting  { background: Vec<Option<f64>> },
}

struct Scan {
    points: Vec<(f64, f64, bool)>,
}

fn main() {
    env_logger::init();

    let mut lidar = match LD19::open(PORT) {
        Ok(l) => l,
        Err(e) => {
            eprintln!("Failed to open {PORT}: {e}");
            std::process::exit(1);
        }
    };

    let mut window = Window::new(
        "LD19 Lidar — Motion Detector  (ESC to quit)",
        WIDTH,
        HEIGHT,
        WindowOptions::default(),
    )
    .unwrap();
    window.set_target_fps(20);

    let mut buf = vec![BG_COLOR; WIDTH * HEIGHT];
    let mut scan = Scan { points: Vec::with_capacity(500) };
    let mut last_dir: u16 = u16::MAX;

    let mut phase = Phase::Calibrating {
        started: Instant::now(),
        samples: vec![Vec::new(); ANGLE_BINS],
    };

    println!("Calibrating for {}s — keep the scene static...", CALIBRATION.as_secs());

    while window.is_open() && !window.is_key_down(Key::Escape) {
        for raw in lidar.poll() {
            let wrapped = last_dir != u16::MAX
                && raw.dir < last_dir
                && last_dir > DIR_ROUND / 2;
            last_dir = raw.dir;

            if wrapped {
                phase = maybe_finish_calibration(phase);
                redraw(&scan, &phase, &mut buf);
                window.update_with_buffer(&buf, WIDTH, HEIGHT).unwrap();
                scan.points.clear();
            }

            if raw.len == 0 { continue; }
            let meters = raw.len as f64 / 1000.0;
            if meters > MAX_RANGE_M { continue; }
            let deg = raw.dir as f64 / 100.0;
            let bin = (deg.floor() as usize) % ANGLE_BINS;
            let rad = deg.to_radians();
            let x = rad.cos() * meters;
            let y = rad.sin() * meters;

            let is_motion = match &mut phase {
                Phase::Calibrating { samples, .. } => {
                    samples[bin].push(meters);
                    false
                }
                Phase::Detecting { background } => match background[bin] {
                    Some(bg_m) => (meters - bg_m).abs() > MATCH_TOLERANCE_M,
                    None => true,
                },
            };
            scan.points.push((x, y, is_motion));
        }
    }
}

fn maybe_finish_calibration(phase: Phase) -> Phase {
    match phase {
        Phase::Calibrating { started, samples } if started.elapsed() >= CALIBRATION => {
            let background: Vec<Option<f64>> = samples.iter().map(|s| {
                if (s.len() as u32) < MIN_BG_SAMPLES { return None; }
                let mut v = s.clone();
                v.sort_by(|a, b| a.partial_cmp(b).unwrap());
                Some(v[v.len() / 2])
            }).collect();
            let locked = background.iter().filter(|b| b.is_some()).count();
            println!("Calibration done. Locked {} / {} angle bins. Motion detection active.", locked, ANGLE_BINS);
            Phase::Detecting { background }
        }
        other => other,
    }
}

fn redraw(scan: &Scan, phase: &Phase, buf: &mut Vec<u32>) {
    buf.fill(BG_COLOR);

    let cx = WIDTH as f64 / 2.0;
    let cy = HEIGHT as f64 / 2.0;
    let scale = cx.min(cy) / MAX_RANGE_M;

    for r_m in 1..=(MAX_RANGE_M as usize) {
        let r_px = (r_m as f64 * scale) as isize;
        draw_circle(buf, cx as isize, cy as isize, r_px, GRID);
    }
    draw_hline(buf, cy as usize, GRID);
    draw_vline(buf, cx as usize, GRID);

    match phase {
        Phase::Calibrating { started, .. } => {
            for &(x, y, _) in &scan.points {
                plot(buf, cx, cy, scale, x, y, 2, CAL_COLOR);
            }
            let frac = (started.elapsed().as_secs_f64() / CALIBRATION.as_secs_f64()).min(1.0);
            draw_progress_arc(buf, cx as isize, cy as isize, (cx.min(cy) - 10.0) as isize, frac, CAL_COLOR);
        }
        Phase::Detecting { background } => {
            for (bin, dist) in background.iter().enumerate() {
                if let Some(meters) = dist {
                    let rad = (bin as f64).to_radians();
                    let x = rad.cos() * meters;
                    let y = rad.sin() * meters;
                    plot(buf, cx, cy, scale, x, y, 2, STATIC_COLOR);
                }
            }
            for &(x, y, motion) in &scan.points {
                if motion {
                    plot(buf, cx, cy, scale, x, y, 3, MOTION_COLOR);
                }
            }
        }
    }

    fill_dot(buf, cx as isize, cy as isize, 4, ORIGIN);
}

fn plot(buf: &mut Vec<u32>, cx: f64, cy: f64, scale: f64, x: f64, y: f64, r: isize, color: u32) {
    let px = (cx + x * scale).round() as isize;
    let py = (cy - y * scale).round() as isize;
    fill_dot(buf, px, py, r, color);
}

fn set_px(buf: &mut Vec<u32>, x: isize, y: isize, color: u32) {
    if x >= 0 && x < WIDTH as isize && y >= 0 && y < HEIGHT as isize {
        buf[y as usize * WIDTH + x as usize] = color;
    }
}

fn fill_dot(buf: &mut Vec<u32>, cx: isize, cy: isize, r: isize, color: u32) {
    for dy in -r..=r {
        for dx in -r..=r {
            if dx * dx + dy * dy <= r * r {
                set_px(buf, cx + dx, cy + dy, color);
            }
        }
    }
}

fn draw_hline(buf: &mut Vec<u32>, y: usize, color: u32) {
    if y < HEIGHT {
        for x in 0..WIDTH { buf[y * WIDTH + x] = color; }
    }
}

fn draw_vline(buf: &mut Vec<u32>, x: usize, color: u32) {
    if x < WIDTH {
        for y in 0..HEIGHT { buf[y * WIDTH + x] = color; }
    }
}

fn draw_circle(buf: &mut Vec<u32>, cx: isize, cy: isize, r: isize, color: u32) {
    let (mut x, mut y, mut d) = (0isize, r, 1 - r);
    while x <= y {
        for &(px, py) in &[
            (cx+x,cy+y),(cx-x,cy+y),(cx+x,cy-y),(cx-x,cy-y),
            (cx+y,cy+x),(cx-y,cy+x),(cx+y,cy-x),(cx-y,cy-x),
        ] {
            set_px(buf, px, py, color);
        }
        x += 1;
        if d < 0 { d += 2*x + 1; } else { y -= 1; d += 2*(x-y) + 1; }
    }
}


fn draw_progress_arc(buf: &mut Vec<u32>, cx: isize, cy: isize, r: isize, frac: f64, color: u32) {
    let steps = 360;
    let end = (frac * steps as f64) as i32;
    for i in 0..end {
        let a = (i as f64 / steps as f64) * std::f64::consts::TAU - std::f64::consts::FRAC_PI_2;
        let px = cx + (a.cos() * r as f64).round() as isize;
        let py = cy + (a.sin() * r as f64).round() as isize;
        for dx in -1..=1 {
            for dy in -1..=1 {
                set_px(buf, px + dx, py + dy, color);
            }
        }
    }
}
