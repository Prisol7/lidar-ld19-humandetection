# How this program works

A walkthrough for someone who just opened the repo. The goal: take raw bytes
coming out of an LD19 lidar over USB-serial, turn them into a live picture,
subtract the background, cluster what's left, and flag things that look like
humans.

---

## 1. What the LD19 actually sends you

The LD19 is a spinning laser rangefinder. A tiny motor spins a laser + sensor
around maybe 10 times a second. Every few degrees it fires the laser and
measures how long the light takes to bounce back. That gives one **distance
measurement** at one **angle**.

It doesn't send those one at a time — it groups **12 measurements per packet**
and streams them out of the USB cable at **230,400 baud** (bits per second).

Each packet is 47 bytes laid out like this (see
[src/port_buffer/data.rs:1](src/port_buffer/data.rs#L1)):

```
┌──────┬──────┬─────────┬──────────┬─────────────────┬──────────┬───────┬──────┐
│ 0x54 │ len  │  speed  │  angle₀  │ 12 × (dist, cf) │  angleₙ  │ time  │ crc  │
│ 1 B  │ 1 B  │  2 B    │   2 B    │     36 B        │   2 B    │  2 B  │ 1 B  │
└──────┴──────┴─────────┴──────────┴─────────────────┴──────────┴───────┴──────┘
```

- `0x54` — the **head byte**, a sync marker so the parser can find packet
  boundaries in a continuous byte stream.
- `len` — always `12` (low 5 bits), the number of point measurements in this
  packet.
- `speed` — current rotation speed in °/s.
- `angle₀` / `angleₙ` — start and end angles in hundredths of a degree
  (so `36000` == `360°`, hence [DIR_ROUND](src/lib.rs#L7)).
- 12 points, each 3 bytes: 2 bytes of distance in millimeters + 1 byte of
  confidence (0–255).
- `crc` — 8-bit CRC over all the earlier bytes. If it doesn't match, we throw
  the packet away.

### Turning bytes into points

The program reads bytes from the serial port into a fixed-size buffer
(exactly one packet wide). When the buffer fills up it tries to decode
([src/port_buffer.rs:43-55](src/port_buffer.rs#L43-L55)).

Three things can happen:

1. **Valid packet** — head byte + length + CRC all check out. Decode the 12
   points and queue them up.
2. **Garbage, but a head byte exists later in the buffer** — shift the buffer
   forward to that head byte and try again next time. This is how the parser
   resynchronizes if it started mid-stream.
3. **No head byte anywhere** — drop everything, refill the buffer.

For each point we only know `angle₀` (start) and `angleₙ` (end) of the packet.
The 12 individual angles are **linearly interpolated** between them
([data.rs:53-60](src/port_buffer/data.rs#L53-L60)):

```
angle_each = (angleₙ - angle₀) / 11
point[i].angle = angle₀ + i × angle_each
```

If a point's confidence is below `min_confidence`, its distance is zeroed so
the main loop ignores it ([data.rs:90](src/port_buffer/data.rs#L90)).

The net result: `LD19::poll()` returns an iterator of
`Point { len: u16 /* mm */, dir: u16 /* hundredths of a degree */ }`.

---

## 2. Polar to cartesian

Each `Point` from the lidar is **polar** (distance + angle from the device's
forward direction). Humans reading a 2D map want **cartesian** (x, y).

[src/bin/main.rs](src/bin/main.rs#L87-L90) converts it:

```rust
let meters = raw.len as f64 / 1000.0;
let deg    = raw.dir as f64 / 100.0;
let rad    = deg.to_radians();
let x = rad.cos() * meters;
let y = rad.sin() * meters;
```

Points farther than `MAX_RANGE_M = 4.0` meters get dropped. Points with
`len == 0` (bad reading or low confidence) get dropped.

---

## 3. Detecting a full rotation

The program needs to know when one 360° sweep is done so it can redraw the
window. It does this by watching for **angle wraparound**: if the current
reading's angle is *smaller* than the previous one, and the previous one was
past the halfway point, we just crossed 0°
([main.rs:78-82](src/bin/main.rs#L78-L82)).

When that happens: finalize the scan (classify + draw), then start the next
scan.

---

## 4. The 30-second calibration — "what's normally there?"

The lidar sees *everything* — walls, furniture, the person standing in the
corner, the dog. We only care about things that **shouldn't** be there. So we
first learn what's normal.

### What calibration does

The program enters `Phase::Calibrating` for `CALIBRATION = 30` seconds. During
this time:

- The world is divided into **360 angular bins**, one per degree
  ([ANGLE_BINS](src/bin/main.rs#L12)).
- For every incoming point, we figure out which bin its angle falls into and
  **append its distance** to that bin's list of samples.

After 30 seconds each bin has maybe 300+ distance readings from that angle.

### Building the background

When calibration ends ([maybe_finish_calibration](src/bin/main.rs#L136)):

- For each bin, sort the samples and take the **median** — a single
  "typical distance" for that angle.
- Bins with fewer than `MIN_BG_SAMPLES = 5` hits are marked `None` (not
  enough data to trust).

Why the median instead of the average? Because occasionally a person walks
through during calibration, or a reading randomly glitches to zero. The median
ignores outliers; the average would be pulled by them.

The final product is `background: Vec<Option<f64>>` — 360 slots, each holding
"the distance the wall is at this angle" or `None`.

### After calibration: motion detection

Every new point is compared to its bin's background value
([main.rs:121-124](src/bin/main.rs#L121-L124)):

```rust
Some(bg_m) => (meters - bg_m).abs() > MATCH_TOLERANCE_M,  // moving
None       => true,                                       // no baseline → treat as motion
```

If the new reading is more than `MATCH_TOLERANCE_M = 0.15 m` (15 cm) off the
background, the point is flagged **motion**. Otherwise it's **static** (part
of the wall / furniture) and gets drawn as a black dot.

This is deliberately simple. It assumes the lidar doesn't move and the
environment doesn't rearrange itself. Move a chair after calibration and
you'll get ghost motion at that angle forever (until the next restart).

---

## 5. Human detection pipeline

The interesting part. After each full rotation, we run this pipeline on the
motion points (see [src/detect.rs](src/detect.rs)):

```
motion points → DBSCAN clustering → feature extraction → classifier → tracker
```

### Step A — DBSCAN clustering ([detect.rs:84](src/detect.rs#L84))

A moving person shows up as **a bunch of adjacent motion dots**, not one.
DBSCAN is a clustering algorithm that groups points that are close together
while ignoring loners.

Two parameters control it:

- `DBSCAN_EPS = 0.10 m` — two points "connect" if they're within 10 cm.
- `DBSCAN_MIN_PTS = 4` — a cluster needs at least 4 connected points, else
  it's noise.

The algorithm uses a **k-d tree** (`kdtree` crate) so the "who's within 10 cm
of this point?" query is fast — O(log n) instead of checking every point.

Output: a list of clusters, each a group of points.

### Step B — Geometry features ([detect.rs:147](src/detect.rs#L147))

A cluster is just a bag of points. We extract 4 numbers that describe its
shape:

1. **`width`** — the longest straight-line distance between any two points in
   the cluster. This is the widest chord through the blob.
2. **`depth`** — how far the blob bulges *away* from that chord. We draw a
   line between the two widest points and measure the max perpendicular
   distance to any other point.
3. **`curvature`** — fit a circle to the points and take `1 / radius`
   ([fit_circle_curvature](src/detect.rs#L184)). A human torso seen from
   above is roughly circular with radius ~0.20 m, so curvature ≈ 5 m⁻¹.
   A flat wall chunk has curvature close to 0.
4. **`point_count`** — how many points are in the cluster. Larger objects
   return more lidar hits.

### Step C — Classifier ([detect.rs:41](src/detect.rs#L41))

Two modes:

- **Rule-based (default)**: the `Classifier::Rule` variant uses hand-tuned
  thresholds
  ([rule_predict](src/detect.rs#L79)):
  - `width` between 0.30 and 0.60 m (a torso, not a chair leg or a wall),
  - `1/curvature` (i.e. fitted radius) between 0.15 and 0.25 m (torso-shaped),
  - at least `DBSCAN_MIN_PTS` points.
- **ML (optional)**: if a file `human_rf.bin` exists in the working directory,
  it's loaded as a `smartcore` `RandomForestClassifier` and used instead. The
  4 features go in, a 0/1 label comes out. Training the model is left to you
  using the logged CSV data.

### Step D — CSV logging ([detect.rs:237](src/detect.rs#L237))

Every cluster's features get appended to `training_data.csv`:

```
width,depth,curvature,point_count,label
0.4203,0.1812,4.9211,42,
...
```

The `label` column is blank. You fill it in by hand (1 = human, 0 = not),
then you can train a Random Forest, serialize it with `bincode` to
`human_rf.bin`, and the program will pick it up on next start.

### Step E — Sticky tracker ([main.rs:142](src/bin/main.rs#L142))

There's a deliberately-crude tracker layered on top. Once a cluster is
classified human, we remember its centroid. On the next scan, **any** cluster
within `TRACK_MATCH_M = 0.40 m` of a remembered centroid is forced to `is_human
= true` — even if the classifier wouldn't have flagged it this frame.

Why: the classifier is noisy. A person rotating or briefly occluded may fail
the curve check for a frame or two. The tracker smooths over those gaps so
the green smile doesn't flicker.

Why it's bad: it will latch onto *any* moving object that wanders through
the last known centroid. A real fix needs something like a Kalman filter +
Hungarian-assignment tracker (SORT). FIXME comments in the code flag this.

Tracks expire after `TRACK_TTL_SCANS = 30` scans with no nearby cluster.

---

## 6. Rendering

`minifb` gives us a simple pixel buffer window. The renderer
([redraw in main.rs](src/bin/main.rs#L156)) clears to white, draws a grid
(concentric meter rings + crosshair), then layers on:

- **Calibrating phase**: all live points in blue, plus a clock-style progress
  arc.
- **Detecting phase**:
  - Static background dots (the learned median at each angle) in **black**.
  - Motion points (not in human cluster) in red.
  - Human clusters: each point colored on a **blue-to-red gradient** based on
    its distance from the cluster's centroid, with a smile glyph drawn above.
- The origin (the lidar itself) as a yellow dot.

The world-to-screen math:
```
pixel_x = window_width/2  + x_meters × scale
pixel_y = window_height/2 - y_meters × scale    ← Y flipped so +Y is "up"
scale   = half_window / MAX_RANGE_M
```

---

## 7. The main loop, put together

```
open serial port
open window
enter Calibrating phase
loop:
    for each new byte batch from the lidar:
        parse packets → yield Points
        for each Point:
            if angle wrapped past 0°:
                maybe finish calibration
                if Detecting:
                    cluster motion points
                    extract features per cluster
                    classify each cluster (rule or RF model)
                    update sticky tracker
                    append features to training_data.csv
                redraw window
                clear scan buffer
            convert to cartesian
            if Calibrating: record sample in its angle bin
            if Detecting:  compare to background → motion flag
            push (x, y, motion_flag) onto scan buffer
    if user pressed ESC: exit
```

---

## 8. File layout

| File | Purpose |
| --- | --- |
| [src/lib.rs](src/lib.rs) | Crate entry point. Opens the serial port, exposes `LD19::poll()`. |
| [src/port_buffer.rs](src/port_buffer.rs) | Byte-stream → `Point` iterator. Finds packet boundaries, resyncs on garbage. |
| [src/port_buffer/data.rs](src/port_buffer/data.rs) | The 47-byte packet layout, CRC check, angle interpolation. |
| [src/detect.rs](src/detect.rs) | DBSCAN, feature extraction, classifier, CSV logger. |
| [src/bin/main.rs](src/bin/main.rs) | The app. Main loop, calibration, motion detection, tracker, rendering. |

---

## 9. Things that will surprise you later

- **The lidar must be still during calibration.** If you bump it, the
  background model is wrong for the rest of the session. Restart.
- **Moving an object after calibration makes it permanently "motion."** The
  background model doesn't update. This is a design choice, not a bug.
- **The port is hard-coded** to `/dev/ttyUSB0` ([main.rs:6](src/bin/main.rs#L6)).
  If your USB-serial enumerates differently, change it or symlink it.
- **A torso radius of 0.15–0.25 m assumes the lidar is at roughly torso
  height.** If it's on the floor, you're scanning legs; the curvature check
  will reject everyone.
- **Legs are not "convex circles."** In leg-scan setups you'd cluster pairs of
  legs instead — a whole separate detector. Not what this program does.
- **`training_data.csv` has no label column filled in.** The model-based
  classifier won't work until *you* label some rows and train a forest.
