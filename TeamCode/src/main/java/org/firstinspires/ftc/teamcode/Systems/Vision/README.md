# Vision

Reference for the vision stack this package can be built on: **NumPy**, **OpenCV**, **Roboflow datasets**,
**Limelight Python SnapScripts**, **OpenFTC** (EasyOpenCV / VisionPortal / EOCV-Sim) and the **Limelight 3A**.

Part 1 covers what each tool is, why you'd use it, what it costs you, and the details that catch teams out.
Part 2 covers what each tool can do, the core ideas behind it, and a stack of niche strategies.
Part 3 covers object detection in general: task types, classic and learned detectors, and how to measure them.
Part 4 covers the accuracy system: the chain of errors from pixel to field position and how to close each one.
Part 5 covers camera systems: which cameras to know about, and how to choose, place, mount, wire and run them.
Part 6 covers the Limelight Lab: tuning live from the laptop (or with Claude driving the web UI), and keeping pipelines
and SnapScripts in this repo so you can upload them straight from here.

Sources are the official pages: limelightvision.io and docs.limelightvision.io, github.com/OpenFTC (EasyOpenCV,
EOCV-Sim), ftc-docs.firstinspires.org (VisionPortal / AprilTag), roboflow.com and docs.roboflow.com, opencv.org and
numpy.org. They were read in November 2026. Prices and stock change, so check before buying.

---

## How they fit together

```
                 ┌──────────── on the Limelight 3A (its own CPU) ──────────┐
 camera ──► Limelight pipeline: color / AprilTag+MegaTag / neural / Python SnapScript ──► tx, ty, ta, botpose, llpython[]
                                                    ▲ OpenCV 4.10 + NumPy                     │ USB "Ethernet"
                                                    │                                         ▼
 Roboflow dataset ──► train (Limelight trainer / TFRecord) ──► .tflite                    Control Hub (TeamCode)
                                                                                              ▲
 webcam ──► VisionPortal (AprilTagProcessor, ColorBlobLocator, your VisionProcessor) ──────────┘
            built on EasyOpenCV ── OpenCV (Java) ── runs on the Control Hub's 4 A53 cores
            prototype on a PC first in EOCV-Sim / VisionBench
```

There are two places to put the compute:

- **On the Control Hub**, with a webcam and VisionPortal/EasyOpenCV. It's free, but it competes with the control loop
  for the same 4 slow cores.
- **On a Limelight 3A**. Its quad Cortex-A72 does the vision, and the hub only reads the results over USB.

---

# Part 1: What each tool is, why use it, costs and benefits

## NumPy

**What it is:** the core Python package for numeric computing. At its centre is `ndarray`, a fixed-size, n-dimensional
array where every element has the same type. Maths runs over the whole array at once in precompiled C. NumPy's own
docs give two reasons it's fast: **vectorization** (no Python loops; the loop runs in C) and **broadcasting** (an
operation stretches across arrays of compatible shapes without making copies).

**Why we care:** in Python, an OpenCV image *is* a NumPy array, shape `(height, width, channels)`, `dtype uint8`.
Any Limelight SnapScript that does more than call `cv2` is NumPy code: masks, pixel counts, statistics, filtering
over time.

| | |
|---|---|
| **Cost** | Free (BSD license). Already installed on the Limelight beside OpenCV. Not available on the Control Hub, where vision is Java. |
| **Benefits** | `c = a * b` runs at close to C speed. Whole-image boolean logic takes one line. It's the common language between `cv2`, the Limelight runtime and every Python data tool you'd use to study logs. |
| **Downsides** | Arrays have a fixed size: "growing" one makes a new copy, so a `np.append` inside the per-frame loop allocates every frame. Silent `uint8` overflow (see Part 2). Easy to write a Python `for` loop over pixels without noticing, which is around 100× slower. |
| **Key characteristics** | One `dtype` per array. Slicing gives views, not copies. C-contiguous memory by default. Indexing is `[row, col]`, which is `[y, x]`. |
| **Use purposes here** | SnapScript post-processing; packing the `llpython` output; offline analysis of shot, pose and vision logs; building HSV threshold tables from sample pixels. |

## OpenCV

**What it is:** the Open Source Computer Vision Library. It has more than 2,500 algorithms, is maintained since 2000
by the non-profit Open Source Vision Foundation, and is **Apache 2** licensed from 4.5.0 (3-clause BSD at 4.4.0 and
below). It's C++ at heart, with Java bindings (used by EasyOpenCV and VisionPortal on the robot) and Python bindings
(used in Limelight SnapScripts).

**Why we care:** every vision path on this robot uses OpenCV underneath. That includes the Limelight's own pipelines
and Python, EasyOpenCV, and VisionPortal's `VisionProcessor` frames. EasyOpenCV and the Limelight both ship
**OpenCV 4.10**, so an algorithm written in one carries over to the other almost line for line.

| | |
|---|---|
| **Cost** | Free. The cost is CPU: on the Control Hub, every millisecond of OpenCV is a millisecond the drive and turret loop doesn't get. |
| **Benefits** | Classic vision is **deterministic, explainable and fast**: thresholding, contours, moments, morphology, `solvePnP`, undistortion. There's no training data, and a bad result traces back to one tunable number. |
| **Downsides** | Hand-tuned colour thresholds break when the lighting changes (venue lights, sunlight, field glare). In Java, a `Mat` holds native memory the garbage collector can't see, so forgetting `release()` leaks it, and "allocate per frame" eventually crashes the app. |
| **Key characteristics** | `Mat` is a reference-counted header over a data buffer, and `submat` shares that buffer. 8-bit HSV hue runs **0–179**, not 0–359. The Python API takes and returns NumPy arrays; the Java API uses `Mat`, `MatOfPoint` and so on. |
| **Use purposes here** | Colour blob detection for game pieces, contour filtering, centroid and angle for turret or intake alignment, distance from target size or height, pose from known geometry (`solvePnP`), lens undistortion. |

## Roboflow datasets

**What it is:** a hosted platform to collect, label, version, preprocess, augment, train and export computer vision
datasets. **Roboflow Universe** is its public catalogue, with more than 1B images, 1M datasets and 250k models,
where FTC teams often post game-piece datasets.

**Why we care:** it's the practical way to feed the **Limelight neural detector or classifier**. You label in
Roboflow, export **TFRecord**, train with Limelight's trainer, and upload an 8-bit `.tflite` file with its labels to
the Limelight.

| | |
|---|---|
| **Cost** | **Free tier:** 10 credits per month (about 30 trainings or about 80k hosted inferences). **Core:** $39 per month for 20 credits. Workspaces are private by default, with optional sharing to Universe. Bounding-box-only augmentations are a premium feature. You can label and export for free and train elsewhere, such as the Limelight trainer. |
| **Benefits** | Recognises things colour can't (shape, partial occlusion, mixed colours). Versioned datasets you can rebuild later. Built-in labelling tools, health check, many export formats. A head start from Universe datasets. |
| **Downsides** | Neural inference on the Limelight 3A runs **on the CPU** (there's no Coral accelerator on the 3A), so it's far slower than a colour pipeline. Labelling takes time. A model is only as good as how much its training images look like match conditions. |
| **Key characteristics** | **Preprocessing** applies to *all* splits (train, valid, test): auto-orient, resize, grayscale, auto-contrast, isolate objects, static crop, tile, modify classes, filter nulls. **Augmentation** is generated *offline* and only for the *train* split; a 3× multiplier on 100 images split 70/20/10 gives about 210/20/10. Versions are frozen snapshots. |
| **Use purposes here** | Game-piece detection when colour isn't unique; classifying the state of the field or a goal; checking what's in our intake from a camera. |

## Python SnapScripts (Limelight)

**What it is:** a pipeline type on the Limelight where **you write the vision in Python**, with OpenCV 4.10 and NumPy
on the Limelight's own processor. The camera calls your `runPipeline(image, llrobot)` on every frame. It returns
`(largestContour, image, llpython)`:

- `largestContour` drives the normal Limelight targeting, so you still get `tx`, `ty` and `ta`.
- `image` is what the stream shows.
- `llpython` is a list of numbers sent back to the robot.

The robot sends numbers *in* with `updatePythonInputs(double[])`; they arrive as `llrobot`. It reads the outputs
with `LLResult.getPythonOutput()`.

**Why we care:** it's full custom vision **without spending any Control Hub CPU**, and you don't need to rebuild the
APK to change it: edit the script in the web UI and it runs straight away.

| | |
|---|---|
| **Cost** | Free with a Limelight. The cost is compute: Python is slower than the built-in C++ pipelines, so a heavy script drops below 90 fps. |
| **Benefits** | Any OpenCV and NumPy logic. A two-way number channel with the robot (`llrobot` / `llpython`). Edit on the fly. Its contour still feeds the standard crosshair targeting. |
| **Downsides** | Runs on the Limelight's Python runtime only: you can't pip-install packages, and what's available is what Limelight ships (OpenCV, NumPy). Debugging is through the stream image and outputs. It's one pipeline slot, so it can't run at the same time as an AprilTag pipeline. |
| **Key characteristics** | The image comes in as BGR (the Limelight example converts with `cv2.COLOR_BGR2HSV`). The script is loaded once, so module-level variables keep their values between frames. `llpython` is a flat list of floats. |
| **Use purposes here** | Things the built-in colour pipeline can't do, such as two colours at once, picking the *best* blob instead of the largest, counting game pieces, or a robot-driven ROI; sending several targets in one frame; filtering over time on the camera. |

## OpenFTC: EasyOpenCV, VisionPortal, EOCV-Sim

**What it is:**

- **EasyOpenCV (EOCV):** OpenFTC's library that pipes webcam or phone-camera frames into an OpenCV pipeline on the
  Robot Controller. It has been **bundled in the FTC SDK since 8.2** (needs SDK 9+ now) and uses **OpenCV 4.10** as
  of v1.7.3.
- **VisionPortal:** FIRST's higher-level API built on top of EOCV. It runs one or more `VisionProcessor`s (the
  built-in `AprilTagProcessor`, colour locators, or your own) on one camera or several. It handles the camera
  controls (exposure, gain, focus, white balance), CPU and USB-bandwidth management, and the live view.
- **EOCV-Sim (now VisionBench):** a PC tool that runs your pipeline against images, videos or a webcam. You tune it
  on a laptop and copy-paste it to the robot. It also has a node-based editor (VisionGraph).

**Why we care:** it's the no-extra-hardware path, needing only a cheap UVC webcam. It's also the only path for
AprilTag pose *on the hub*, and the only way to run several processors on one camera at once.

| | |
|---|---|
| **Cost** | Free. A UVC webcam costs roughly $20–40. The real cost is **Control Hub CPU and RAM**: the 4 A53 cores are shared with the RC app and our loop, so a heavy pipeline adds loop time and jitter, which shows up in HardwareMonitor's loop timing. |
| **Benefits** | No extra controller to power or wire. AprilTag, colour and custom processors run together. Pose, range and bearing for AprilTags out of the box, with field-relative robot pose since SDK 10. Prototype quickly in EOCV-Sim. |
| **Downsides** | Uses the CPU the control loop needs. Frames arrive as **RGBA (EOCV) or RGB**, not BGR, which catches out anyone copying Python examples. Raw EOCV APIs were broken in OnBot Java on SDK 9.0 (port to a `VisionProcessor`). Memory leaks crash the app mid-match. |
| **Key characteristics** | `OpenCvPipeline.processFrame(Mat)` / `VisionProcessor.processFrame(Mat, long captureTimeNanos)`. `init(...)` runs once, so allocate there. Async camera open and close. MJPEG stream format for higher resolutions over USB. Annotate with `onDrawFrame(Canvas, ...)` instead of drawing on the frame. Pause the viewport to save CPU. Built-in memory-leak detector. |
| **Use purposes here** | AprilTag pose to correct Pinpoint drift; colour blob detection for intake assist; a backup camera if the Limelight disconnects. |

## Limelight 3A

**What it is:** a self-contained smart camera for FTC.

- **Price:** $189 (shown as sold out when checked).
- **Sensor:** OV5647 colour rolling-shutter, 640×480 at 90 fps. Field of view is 54.5° horizontal by 42° vertical.
- **Lighting:** **no LEDs**.
- **Compute:** quad Cortex-A72 at 1.5 GHz.
- **Power and connection:** **USB-C only**, 4.1–5.75 V, about 4 W. It plugs into the Control Hub's **USB 3.0 port**
  and shows up as an Ethernet device.
- **Size:** 72 × 48 × 16.8 mm, 0.20 lb.
- **Setup:** through a web UI (`limelight.local:5801`).
- **Pipelines:** **10 hot-swappable** ones, using colour, AprilTag with MegaTag1 or MegaTag2, a neural detector or
  classifier (CPU), barcodes, or Python SnapScripts.
- **Limit:** the REV Control Hub supports **only one** Limelight 3A.

**Why we care:** it moves all the vision work off the hub and gives you a field-relative robot pose (`botpose`)
from AprilTags, fused with our IMU yaw (MegaTag2), in a few lines of Java.

| | |
|---|---|
| **Cost** | $189. It takes the USB 3.0 port and about 4 W from the hub. Only one per Control Hub. Weight and mounting space. |
| **Benefits** | Zero hub CPU for vision. A 90 fps colour pipeline. AprilTag at 20 fps at 1280×960 or 50 fps at 640×480 (2× downscale). MegaTag2 pose that stays stable at long range. Tune everything in a browser with no redeploy. 10 pipelines you can switch from code. |
| **Downsides** | Rolling shutter, so fast spins skew the image. No LEDs, so you depend on field lighting. A neural net on the CPU is slow. A USB device can drop out from ESD or a loose cable mid-match. `pipelineSwitch` is fire-and-forget, so you have to confirm it took effect yourself. |
| **Key characteristics** | `LLResult`: `getTx()/getTy()/getTa()`, `getBotpose()/getBotpose_MT2()`, `getStaleness()`, capture and targeting latency, fiducial and colour result lists, `getPythonOutput()`. Java API: `setPollRateHz`, `pipelineSwitch`, `start/stop`, `updateRobotOrientation(yaw)`, `updatePythonInputs`. ChArUco camera calibration. Snapshots for gathering data. |
| **Use purposes here** | Field-relative pose to correct Pinpoint drift (MegaTag2); goal and AprilTag aiming for the turret as a check on odometry aim; colour targeting of game pieces; Python for anything custom. |

### Which to pick (for this robot)

| Need | Best fit | Why |
|---|---|---|
| Correct odometry drift during a match | **Limelight MegaTag2** | Fuses our yaw, steady at range, zero hub CPU. |
| Same, with no budget | **VisionPortal AprilTagProcessor** with a webcam | Free, but it costs loop time, so decimate and lower the resolution. |
| Find coloured game pieces fast | **Limelight colour pipeline** | 90 fps, C++. "Do the simple thing first." |
| Game pieces that colour can't separate | **Roboflow → Limelight neural detector** | Learns shape. Accept the lower fps. |
| Custom logic (best blob, counts, robot-driven ROI) | **Python SnapScript** | Custom code with no hub CPU. |
| Prototyping any OpenCV idea | **EOCV-Sim / VisionBench** or a Python notebook | Fast iteration on recorded frames. |

---

# Part 2: Abilities, core concepts and niche strategies

## NumPy

**Abilities:**

- Vectorized maths, logic and comparisons over whole images.
- Boolean masks and fancy indexing.
- Reductions: `sum`, `mean`, `count_nonzero`, `argmax`, `percentile`.
- Linear algebra: `linalg.solve`, `lstsq` for fitting.
- Random numbers for Monte Carlo.
- FFT for spotting vibration in logs.
- Fast I/O: `np.save`/`np.load`, `loadtxt`.

**Core concepts:**

- **ndarray = buffer + shape + strides + dtype.** A slice like `img[100:200, 50:150]` is a *view* over the same
  memory, so writing to it writes into the image. Use `.copy()` when you need your own data.
- **Broadcasting:** shapes are compared from the right. `img - np.array([b, g, r])` subtracts one colour from every
  pixel without a loop.
- **Vectorization:** push the loop into C. If you write `for` over pixels, you're doing it wrong.

**Niche strategies:**

1. **`uint8` wraps, `cv2` saturates.** `np.uint8(200) + np.uint8(100)` gives **44**, while `cv2.add` gives **255**.
   For brightness or contrast maths, use `cv2.add`/`cv2.addWeighted`, or cast first with
   `img.astype(np.int16)`, then `np.clip(..., 0, 255).astype(np.uint8)`.
2. **Shapes are `(h, w)` but OpenCV sizes are `(w, h)`.** `img.shape[:2]` is `(rows, cols)`, but `cv2.resize(img, (w, h))`
   takes width first. Points from contours are `(x, y)`, but you index pixels as `img[y, x]`.
3. **Count, don't contour, when you only need "how much".** `np.count_nonzero(mask)` on an `inRange` mask measures
   fill in microseconds. It works well for "is there a game piece in the intake ROI?" without `findContours`.
4. **Fast centroid without moments:** `ys, xs = np.nonzero(mask)`, then `xs.mean()` and `ys.mean()`. Use
   `np.median` instead for a centroid that resists stray pixels.
5. **Preallocate in SnapScripts.** Put buffers at module level (for example `kernel = np.ones((5,5), np.uint8)`) and
   use `out=` arguments (`np.multiply(a, b, out=buf)`, `cv2.inRange(hsv, lo, hi, dst=mask)`) so frames don't
   allocate.
6. **Pick thresholds from data.** Gather the HSV pixels of a game piece across snapshots, then set
   `lo = np.percentile(px, 2, axis=0)` and `hi = np.percentile(px, 98, axis=0)`. That replaces guessing with sliders.
7. **Smooth over time on the camera.** Module-level state persists, so an exponential filter (`s = 0.7*s + 0.3*x`)
   or a short ring buffer with `np.median` steadies `llpython` outputs before they reach the robot.
8. **`np.ascontiguousarray`** fixes "Layout of the output array is incompatible" errors from `cv2` after you have
   transposed or sliced with steps.
9. **Use `float32`, not `float64`, for large intermediate arrays.** It's half the memory bandwidth, and OpenCV prefers it.
10. **Analysing logs offline:** load HardwareMonitor CSVs, vision exports or shot logs with `np.loadtxt` or pandas,
    then `np.polyfit` distance against `ty` to build a range table, and `np.fft.rfft` the turret error to find
    vibration frequencies.

## OpenCV

**Abilities:**

- Colour spaces: BGR/RGB, HSV, Lab, YCrCb, Gray.
- Thresholding: `inRange`, Otsu, adaptive.
- Filtering: Gaussian, median, bilateral.
- Morphology: erode, dilate, open, close.
- Shapes: contours, moments, `minAreaRect`, `fitEllipse`, `approxPolyDP`, `convexHull`.
- Edges and lines: Canny, Hough lines and circles.
- Templates and features: template matching, ORB features.
- Geometry: camera calibration, `undistort`, `solvePnP` and `projectPoints`.
- Optical flow.
- DNN inference: the `dnn` module.

**Core concepts:**

- **The classic pipeline:** convert colour → threshold to a mask → clean it with morphology → find contours →
  filter by geometry → measure (centroid, angle, size) → turn into robot units.
- **Geometry filters** are what make thresholding robust:
  - area,
  - aspect ratio (`w/h` of `boundingRect`),
  - **extent** (contour area ÷ bounding box area),
  - **solidity** (contour area ÷ convex hull area),
  - circularity (`4πA/P²`).
- **Camera model:** a pinhole with intrinsics (`fx, fy, cx, cy`) plus distortion. Pose from known 3-D points comes
  from `solvePnP`.

**Niche strategies:**

1. **Red wraps around hue 0.** In 8-bit HSV, red sits at about 0–10 *and* about 170–179. Threshold both ranges and
   combine them with `bitwise_or`. Or rotate hue by 90 first (`(h + 90) % 180`) so red becomes one band.
2. **Saturation and value floors matter more than hue.** Most false positives are grey or white field elements with
   a hue that happens to match. Raise the S minimum first.
3. **Lab is steadier than HSV under changing light** for some colours. The `a` channel separates red and green, and
   `b` separates blue and yellow, apart from brightness (`L`).
4. **Shrink before processing.** Halving the resolution with `pyrDown` cuts the work to about ¼. Most FTC targets are
   big enough. Scale the coordinates back up afterwards.
5. **Use an ROI.** Only process the part of the frame where targets can actually appear (`submat` in Java, a slice
   in Python). Below the horizon for floor pieces, above the drivetrain for goals.
6. **Open, then close.** Opening removes speckles, then closing fills holes in the blob. Use an elliptical kernel
   (`getStructuringElement(MORPH_ELLIPSE, ...)`) so corners don't get blocky.
7. **Distance from `ty`** for a camera at a fixed height and pitch:
   `d = (h_target − h_cam) / tan(pitch_cam + ty)`. Calibrate `pitch_cam` by measuring at two known distances, and
   don't trust the CAD angle.
8. **Distance from size** (`ta` or pixel width) changes with the square of distance and gets noisy up close. Prefer
   the angle method, and use size only to check it.
9. **`minAreaRect` angle conventions changed in OpenCV 4.5.1** (to the range (0, 90]). Code copied from older posts
   gives angles that are wrong by 90°. Work the angle out from the box points yourself if it matters.
10. **`findContours` return values differ by version.** OpenCV 3 returns `(img, contours, hierarchy)`; 4.x returns
    `(contours, hierarchy)`. Both Limelight and EOCV use 4.10. Use `RETR_EXTERNAL` and `CHAIN_APPROX_SIMPLE` unless
    you need holes.
11. **Low exposure is better than blur filters.** Motion blur from a spinning robot can't be filtered out. Cut
    exposure and add gain instead (VisionPortal `ExposureControl` / `GainControl`; Limelight exposure, black level
    and sensor gain).
12. **Java `Mat` hygiene:** allocate every `Mat` once, in `init`. Reuse them in `processFrame`, and `release()` any
    temporary you do make. `MatOfPoint` lists from `findContours` must be released too, or cleared and reused.
    Leaked native memory doesn't trigger garbage collection, so the app dies without warning a few minutes into a
    match.
13. **Calibrate the lens before `solvePnP`.** Wide webcams have strong barrel distortion, which bends pose
    estimates near the image edges.

## Roboflow datasets

**Abilities:**

- Upload images and video (with frame sampling).
- Annotate: boxes, polygons, classification, Label Assist and model-assisted labelling.
- Health check: class balance, image sizes, annotation heatmaps.
- Train, valid and test splits.
- Preprocessing and augmentation.
- Versioning.
- Export in many formats: COCO, YOLO, Pascal VOC, **TFRecord**, ...
- Training and hosted inference.
- Universe: browse and fork public datasets and models.

**Core concepts:**

- **Dataset version:** a frozen combination of images, labels, splits, preprocessing and augmentation. Train from
  versions so a result can always be reproduced.
- **Preprocessing applies everywhere, augmentation to train only.** Validation and test stay real, so they measure
  the truth.
- **The model learns what's in the data, including the mistakes.** Labels that disagree with each other are worse
  than fewer labels.

**Niche strategies:**

1. **Shoot your dataset with the deployment camera.** Use Limelight snapshots at your real exposure, resolution and
   mounting angle. A phone-camera dataset will mislead you about how well the model works.
2. **Train a baseline with no augmentation first.** Roboflow's docs advise it. Then add one augmentation at a time
   and keep it only if the validation score goes up.
3. **No hue augmentation for classes defined by colour.** If red and blue pieces are separate classes, hue shifts
   turn one into the other. Brightness and exposure are fine.
4. **Beware split leakage from video.** Neighbouring frames are near-duplicates. If one lands in train and its
   neighbour in test, test scores look great but the model is overfitted. Sample sparsely, or split by *video* or
   session before sampling.
5. **Add null images,** with no target in them: empty field, robot parts, people's clothes, the other alliance's
   colours. They teach the model when *not* to fire. Keep them in, and don't let "filter null" remove them.
6. **One box rule, applied everywhere.** Decide whether a box covers only the visible part or the whole object when
   occluded, and keep to it. Use lowercase, simple labels and as few classes as possible (Limelight's advice).
7. **Tile for small objects.** Distant pieces take up a few pixels. Tiling during preprocessing gives them more
   pixels per object in training.
8. **Resize to the model's real input size.** Limelight detectors run at a fixed input size, so make the dataset
   match, and don't stretch images in a way that distorts the aspect ratio the camera really produces.
9. **Start from Universe, then refine.** Fork a public FTC dataset for the season, then add a few hundred of your own
   frames from the real field. Your data is what fixes the domain gap.
10. **Spend credits carefully.** The free tier is 10 credits a month. Label and export there, and train on the
    Limelight trainer (TFRecord export → Google Colab notebook: about 4,000 steps for a quick test, 20,000 for
    accuracy, up to about 5 hours) instead of using Roboflow training credits.
11. **Gather data from deliberately hard situations:** venue-style lighting, glare, motion blur, partial occlusion,
    pieces touching each other or the wall, pieces held in another robot's intake.

## Python SnapScripts

**Abilities:**

- Any OpenCV 4.10 / NumPy processing on the Limelight.
- Draw on the output stream.
- Return a contour for standard `tx/ty/ta` targeting.
- Send a list of numbers each way: robot → `llrobot`, and `llpython` → robot.
- Keep state between frames.

**Core concepts:**

```python
import cv2
import numpy as np

kernel = np.ones((5, 5), np.uint8)   # module level: built once, persists
smoothed = 0.0

def runPipeline(image, llrobot):
    global smoothed
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (100, 120, 60), (130, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest = np.array([[]])
    llpython = [0, 0, 0, 0]
    if contours:
        largest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)
        smoothed = 0.7 * smoothed + 0.3 * (x + w / 2)
        llpython = [1, smoothed, y + h / 2, len(contours)]
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 255), 2)
    return largest, image, llpython
```

Robot side (Java):

```java
limelight.updatePythonInputs(new double[] {mode, roiTop, 0, 0});
LLResult result = limelight.getLatestResult();
double[] out = result.getPythonOutput();
```

**Niche strategies:**

1. **Return the contour you actually want.** The `largestContour` slot doesn't have to be the largest. Return your
   best-scoring blob, and the built-in `tx/ty`, crosshair and latency data follow it for free.
2. **Use `llrobot` as a command channel:** mode switches (which colour, which ROI), the robot's heading or turret
   angle for compensation, or a "take snapshot now" flag. That's one script with many behaviours, without switching
   pipelines.
3. **Put a validity flag and a frame counter in `llpython`** (`[valid, counter, ...]`). The robot can then reject
   repeated or stale frames even when `isValid()` says true.
4. **Keep `llpython` the same fixed length** so robot code indexes it the same way every time. Zero-fill when there's
   no target.
5. **Send several targets in one frame:** flatten the top N blobs (`x, y, area` each) into `llpython` and pick
   between them on the robot using pose and path context.
6. **Draw less, and only for debugging.** Drawing and text cost real time at 90 fps. Wrap them in an `llrobot` debug
   flag.
7. **Profile with `time.perf_counter()`** around each stage, and report the slowest stage in a spare `llpython` slot.
8. **Downscale first** (`cv2.resize` or `pyrDown`) and scale coordinates back, unless you need sub-pixel accuracy.
9. **Prototype on recorded snapshots** in a desktop Python with the same `runPipeline` signature. Write a three-line
   harness that loads JPGs and calls it, then paste the script into the web UI.

## OpenFTC (EasyOpenCV / VisionPortal / EOCV-Sim)

**Abilities:**

- **Cameras:** webcams and phone cameras, several cameras at once, async open and close, MJPEG or YUY2 streams,
  exposure, gain, focus and white-balance control.
- **Processors:** several at once on one camera, each enabled or disabled while running (`setProcessorEnabled`).
- **AprilTag:** pose (`ftcPose`: x, y, z, yaw, pitch, roll, range, bearing, elevation) and field robot pose given
  the camera's mounting pose.
- **Display:** live view and dashboard stream, annotation with a Canvas.
- **Debugging:** memory-leak detector.
- **Prototyping:** EOCV-Sim / VisionBench pipelines on a PC.

**Core concepts:**

```java
public final class BlobProcessor implements VisionProcessor {

    private final Mat hsv = new Mat(), mask = new Mat();
    private final Scalar low = new Scalar(100, 120, 60), high = new Scalar(130, 255, 255);
    private volatile double centerX = -1;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, low, high, mask);
        Moments m = Imgproc.moments(mask, true);
        centerX = m.m00 > 500 ? m.m10 / m.m00 : -1;
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}

    public double getCenterX() {
        return centerX;
    }
}
```

Points to notice in that example:

- `Mat`s are fields, so nothing is allocated per frame.
- The result is `volatile` because the camera thread writes it and the loop thread reads it.
- The frame is never drawn on.

**Niche strategies:**

1. **Don't draw on the frame in `processFrame`.** Every processor shares the same `Mat`, so drawing on it corrupts
   the input for the next processor (for example AprilTag). Draw in `onDrawFrame` with the Canvas, and use
   `scaleBmpPxToCanvasPx`.
2. **Check the channels before converting:** EOCV gives RGBA and VisionPortal RGB. `COLOR_BGR2HSV` on RGB swaps red
   and blue, so your "blue" threshold finds red pieces.
3. **AprilTag decimation** trades range for speed. A higher `setDecimation` gives more fps and less CPU but detects
   tags at shorter range. Switch it while running: high when close (or in TeleOp near the goal), low when you need
   long-range detection.
4. **Turn processors off when not needed** (`setProcessorEnabled(proc, false)`), and stop the live view
   (`stopLiveView`) or stream in a match. Every disabled processor gives CPU back to the control loop. Watch the loop
   time in HardwareMonitor before and after.
5. **Use MJPEG at high resolutions.** YUY2 at 1280×720 can exceed the USB 2.0 bandwidth and drop the frame rate. Use
   the lowest resolution that still finds your targets.
6. **Set manual exposure and gain** (`ExposureControl.Mode.Manual`, a few ms, plus `GainControl`) to cut motion blur
   when spinning. Auto exposure hunts under venue lights.
7. **Lens intrinsics:** AprilTag pose needs them. The SDK has calibrations for common webcams. For anything else,
   calibrate and pass `setLensIntrinsics(fx, fy, cx, cy)`, or every range reading is scaled wrong.
8. **Camera pose for robot pose:** give the AprilTag processor the camera's position and orientation on the robot.
   The SDK then returns a field-relative robot pose to fuse with Pinpoint.
9. **Account for `captureTimeNanos`.** The frame is older than "now". Rewind your pose history to the capture time
   before fusing a vision pose, especially while driving.
10. **Close the portal on stop.** A camera left open can block the next OpMode from getting it. Our OpMode stop hooks
    (`MonitorOpModeHook`) are the place for it.
11. **Prototype in EOCV-Sim / VisionBench** against recorded field videos, then paste the pipeline in unchanged. Keep
    it free of SDK-only calls so it runs in both places.
12. **Watch the memory-leak detector.** If it fires, a `Mat` is being created each frame. Fix that before a match,
    not during one.

## Limelight 3A

**Abilities:**

- **Pipelines:** 10 switchable pipelines. Colour at 90 fps. AprilTag with MegaTag1 (pose from tags alone) and
  MegaTag2 (pose from tags fused with the robot's yaw), at 20 fps at 1280×960 or 50 fps at 640×480. Neural detector
  and classifier on the CPU. Barcodes. Python SnapScripts.
- **Calibration:** ChArUco camera calibration.
- **Data:** snapshots for collecting data.
- **Results:** latency reporting and staleness; per-fiducial results with ID filtering.

**Core concepts:**

```java
Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
limelight.setPollRateHz(100);
limelight.pipelineSwitch(0);
limelight.start();

//each loop
limelight.updateRobotOrientation(Math.toDegrees(heading));
LLResult result = limelight.getLatestResult();
if (result != null && result.isValid() && result.getStaleness() < 100) {
    Pose3D botpose = result.getBotpose_MT2();
}
```

- **tx / ty:** the target's angle from the crosshair, in degrees. **ta** is its area as % of the image.
- **MegaTag2** needs the robot's yaw *every frame*, plus the camera's pose in robot space (set in the web UI). In
  return it stays steady at long range and with a single tag, where MegaTag1 flips around.
- **Staleness:** how old the result is in ms. **Latency** is capture plus targeting time. Both matter when the robot
  is moving.

**Niche strategies:**

1. **Do the simple thing first.** Limelight's own case study: FRC 2056 used a 90 fps colour pipeline instead of a
   neural net. Use the neural net only when colour plus geometry filters fail.
2. **Low exposure, raise black level and gain.** Short exposure kills motion blur. Black-level offset crushes the
   background. Gain gets the brightness back. Do this before touching the HSV thresholds.
3. **Confirm pipeline switches.** `pipelineSwitch` is fire-and-forget: until `result.getPipelineIndex()` matches,
   you're still reading the old pipeline's results. Ignore results until it does.
4. **Gate MegaTag2 updates.** Skip fusion when:
   - there are 0 tags;
   - the robot spins faster than about 360°/s (Limelight's guidance) — our yaw feed lags and the rolling shutter
     skews the frame;
   - the result is stale;
   - the pose is off the field, or more than a few inches from odometry while odometry is healthy.
5. **Filter tags by ID.** Use only the tags whose field positions are fixed and known (the ID filter in the
   pipeline, or check fiducial IDs in code). Tags on moving or random elements poison the pose.
6. **Feed yaw from the same source as odometry** (the Pinpoint heading) so vision and odometry agree on frames. The
   IMU and Pinpoint can drift apart.
7. **Fuse, don't snap.** Blend the vision pose into Pinpoint with a small gain, weighted by tag count and distance,
   instead of `setPose` each frame. Snapping makes the turret aim twitch when the vision is noisy.
8. **Compensate for latency.** Keep a short ring buffer of odometry poses, look up the pose at
   `now − (capture + targeting latency)`, and apply the vision correction *there*, then carry it forward to now.
   Preallocate the buffer, because this is a hot path.
9. **Poll rate:** `setPollRateHz(100)` matches the 90 fps pipeline. A lower rate wastes frames, and a much higher one
   only costs CPU.
10. **ESD and USB drops:** the Limelight is a USB device, so a static discharge or a cable yank can make it vanish
    mid-match (see our `sim esd` disconnect modelling). Strain-relieve the USB-C cable. Code must treat
    `null`, `!isValid()` and growing staleness as "no vision" and fall back to odometry, never stop moving or
    shooting.
11. **One per Control Hub.** If you need two cameras, the second is a webcam through VisionPortal on the hub's
    other USB port.
12. **Use snapshots as the dataset.** Grab snapshots from the robot at match exposure for Roboflow labelling and for
    offline SnapScript or HSV tuning.
13. **Neural deployment checklist:** export TFRecord from Roboflow, train with Limelight's notebook, upload the
    **8-bit** `.tflite` and labels file, and set the runtime to **CPU** (the 3A has no Coral). Expect far below
    90 fps, so use it for occasional decisions, not for closed-loop tracking.
14. **No LEDs:** you depend on field lighting. Tune at an actual event-style light level, and save pipeline variants
    for bright and dim venues so switching is one call.

---

# Part 3: Object detection in general

## The task types

| Task | Output | FTC use | Cost |
|---|---|---|---|
| **Classification** | One label for the whole image or ROI | "Which game piece is in the intake?", "What does the randomisation show?" | Cheapest learned model. Crop the ROI yourself first. |
| **Detection** | Boxes, labels and scores | Find pieces on the floor, find the goal | The FTC workhorse. |
| **Semantic segmentation** | A class for every pixel | Free floor area, tape lines | Heavy. Rarely worth it on FTC hardware. |
| **Instance segmentation** | A mask for each object | Separating pieces that touch | Heavier again. Classic contours plus watershed is often enough. |
| **Keypoints / pose** | Points on each object | Piece orientation for a claw | Heavy. A classic `minAreaRect` angle often does the job. |
| **Fiducials** | Tag ID, corners, full 6-DoF pose | Robot localization (AprilTag 36h11) | Cheap, precise, and they are designed to be found. |
| **Tracking** | Detections with IDs held across frames | Picking the same piece over time, predicting where it will be | Mostly logic, not compute (see below). |

Pick the *least* expressive task that answers the question. Every step down this list costs frames per second
and dataset effort.

## Classic detectors (no training)

- **Colour threshold + contours.** The FTC default. It's deterministic, runs at 90 fps on a Limelight, and you can
  tune it on the spot. It fails when lighting shifts or when the background has the same colour.
- **Colour plus geometry scoring.** Score each blob by area, aspect ratio, solidity, extent, and the expected
  size for its `ty` (row in the image). The highest score wins. This handles most of the "colour isn't unique
  enough" cases without a neural net.
- **Hough circles.** Good for balls when the edges are clean. Sensitive to `param2` and to blur, and slow at full
  resolution.
- **Template matching.** Fast if scale and rotation are fixed (a camera at a fixed spot looking at a fixed
  element). Fails as soon as either changes.
- **Feature matching (ORB + homography).** Finds textured, flat things at any rotation. FTC elements are mostly
  plain colours, so it rarely helps.
- **Background subtraction.** Useless on a moving robot. Works for a fixed camera on a test bench.
- **Fiducials (AprilTag / ArUco).** Black-and-white codes designed to be found, with corner refinement to
  sub-pixel accuracy. Robust to lighting, but they only exist where the game puts them.

## Learned detectors

- **One-stage detectors** (the YOLO family, SSD, EfficientDet-Lite) predict boxes and classes in one pass. That's
  what runs on edge devices like the Limelight.
- **Input size is fixed.** The frame is resized or letterboxed to, for example, 320×320. A piece that's 30 px wide
  in a 640 px frame becomes 15 px at 320. Small, far-off objects are the first to vanish.
- **Confidence scores aren't probabilities.** A 0.6 from one model isn't a 0.6 from another. Choose the
  threshold from a precision–recall curve on *your* validation set (see metrics below).
- **Non-maximum suppression (NMS)** merges overlapping boxes. If two classes share a shape (red and blue pieces),
  use **class-agnostic NMS**, or one piece can come out as two detections, one of each colour.
- **Int8 quantization** (the 8-bit `.tflite` the Limelight wants) is several times faster than float, usually for
  a small accuracy loss. Quantize using representative images from your own dataset.
- **TensorFlow Object Detection (TFOD) is gone from the FTC SDK** (removed in 10.0). Learned detection in FTC now
  means a Limelight neural pipeline, a HuskyLens, or your own model, which isn't practical on the hub's CPU.

## Metrics, and the ones that matter for a robot

- **IoU** (intersection over union): box overlap. A detection counts as correct at IoU ≥ 0.5 in most metrics.
- **Precision** = TP / (TP + FP): of what it found, how much was real.
- **Recall** = TP / (TP + FN): of what was real, how much it found.
- **F1:** the harmonic mean of precision and recall.
- **mAP@0.5 / mAP@0.5:0.95:** precision averaged over recall levels. The second form is the COCO one, averaged over
  IoU 0.5 to 0.95, and it rewards tight boxes.

Those are model metrics. A robot needs **system metrics**, measured on the robot:

| Metric | Why it matters |
|---|---|
| Pickup or shot success rate with vision on vs off | The only number that really counts. |
| False triggers per minute in a full match (other robots, crowd, tape) | One false trigger in auto can cost the run. |
| Field-position error: median and 95th percentile, in inches | The p95 is what drives the claw into the wall. |
| Range at which detection works 90% of the time | Sets where your paths start trusting vision. |
| End-to-end latency, p50 and p95 (capture → action) | Sets how much you need to compensate. |
| Frames per second under match load | The loop-time cost for hub-side vision. |

**Pick the operating point from the use.** An auto pickup that drives into whatever it sees wants **high
precision** (a raised threshold, N-of-M confirmation). A "help the driver line up" overlay can take **high recall**.

## From detection to action

```
detect ──► filter (geometry, ROI, score) ──► localize (pixel → robot → field) ──► track (associate, smooth, predict)
       ──► decide (hysteresis, confirmation) ──► act (Pedro path / turret / intake)
```

**Localizing a floor piece from one camera (ground-plane projection).** A piece sitting on the floor has a known
height, so one pixel is enough to locate it:

1. Undistort the pixel `(u, v)`, then form the ray in the camera frame: `r_c = ((u − cx)/fx, (v − cy)/fy, 1)`.
2. Rotate it into the robot frame with the camera's mounting rotation: `r = R · r_c`. The camera sits at `t`.
3. Intersect with the plane of the piece's centre height `h`: `s = (h − t_z) / r_z`, then `p = t + s · r`.
4. Transform `p` by the robot's pose **at the frame's capture time** to get field coordinates.

The result is only valid for `r_z < 0` (a ray looking down). The error grows quickly as the ray gets close to
horizontal, so ignore detections near the horizon.

**Tracking, briefly:**

- **Associate** new detections with existing tracks by nearest field position, within a gate. Use the Hungarian
  method if several pieces are close together, greedy matching otherwise.
- **Confirm** a track only after N hits in the last M frames (for example 3 of 5) before acting on it.
- **Coast** a track on its prediction for a few missed frames (occlusion by our own intake), then drop it.
- **Smooth** each track with an alpha-beta filter or a small Kalman filter. Pieces on the floor are static, so a
  plain average of field positions works and is the most robust.
- **Commit:** once a pickup starts, lock onto that track's ID. Switching targets halfway causes the
  "zig-zag toward two pieces" failure.

**Decide with hysteresis.** "Target present" turns on at a higher threshold than it turns off, and the turret aim
offset only updates when it changes by more than the measurement noise. Without that, small flickers in
detection become shaking mechanisms.

---

# Part 4: The accuracy system

Vision accuracy is a chain. Every link adds error, and the final number is the sum, so fix the biggest link
first. The numbers below use the Limelight 3A (640×480, 54.5° × 42°), which gives a focal length of about
**621 px**, so **one pixel ≈ 0.09°** at the image centre.

## The error chain

| Link | Typical size | How to shrink it |
|---|---|---|
| **Pixel quantization** | 0.09°/px. At 120 in, 1 px ≈ 0.19 in sideways. | Higher resolution, sub-pixel centroids (moments over the whole mask, not the bounding box). |
| **Segmentation noise** (mask edges flicker) | 1–3 px of centroid jitter | Clean masks (open/close), median over a few frames, a fixed exposure. |
| **Motion blur** | Spin at 180°/s with 10 ms exposure gives about **19.5 px** of smear. At 2 ms, about **3.9 px**. | Short exposure plus gain. Don't trust frames during fast spins. |
| **Rolling-shutter skew** | Rows are captured at different times. A fast spin leans vertical edges. | Global-shutter camera, or reject frames above a spin-rate limit. |
| **Lens distortion** | Several pixels near the edges of wide lenses | Calibrate and undistort, or only use the centre of the image. |
| **Intrinsics error** (fx, cx wrong) | Range scale errors, bearing offsets | Calibrate at the resolution you run. Reprojection error ≲ 0.5 px is good. |
| **Extrinsics error** (camera pose on the robot) | A 1° pitch error at 60 in range, with a 30 in height difference, gives **2.6 in** of range error. At 120 in it's **8.9 in**. | Measure, then refine by putting the robot at known poses and solving. Rigid mounts. |
| **Latency** | 50 ms at 60 in/s is **3 in** of movement. At 180°/s it's **9°** of heading. | Timestamp frames, keep pose history, compensate (below). |
| **Mount flex and vibration** | The camera angle moves with the flywheel and with hits | Stiff mounts, away from the flywheel, checked on the stream with the flywheel running. |
| **Odometry error** (for field-relative results) | Grows over a match | That's why vision corrects odometry. Fuse carefully (below). |

**Where the formulas come from:**

- **Range from angle:** `d = Δh / tan(pitch + ty)`. Its sensitivity to angle error is
  `∂d/∂θ = (Δh² + d²) / Δh` (in inches per radian). Error grows with **d²**, and a *larger* height difference
  `Δh` between camera and target makes far ranges *more* accurate. Put the camera low for a high goal, or the
  other way round.
- **Range from size:** `d = f · W / w_px`. One pixel of width error is a `1 / w_px` fraction of the range. A 5 in
  piece at 100 in is about 31 px wide, so 1 px means about 3% (3 in).
- **Blur:** `blur_px ≈ ω · t_exposure · f`. Translation does the same: `v · t_exposure / d · f`.

## Calibration

1. **Intrinsics.** Use ChArUco on the Limelight, or OpenCV `calibrateCamera` with a checkerboard or ChArUco board
   for webcams. Take 20–40 images that fill the corners of the frame at many tilts. Calibrate **at the resolution
   you run**, because intrinsics change with resolution and binning mode. Keep autofocus off, since focus changes
   intrinsics.
2. **Extrinsics.** Start from CAD for the camera's position and rotation, then correct it from data. Put the robot
   at several surveyed field poses, record the vision pose, and solve for the camera offset that minimizes the
   error. Pitch is the angle that matters most.
3. **Colour.** Fix the white balance and exposure, and record the HSV statistics of each game piece under at least
   two lighting levels (see the NumPy percentile trick in Part 2).
4. **Check the calibration again** after any crash, mount change or camera swap. Keep a quick check routine: the
   robot at a known spot, compare the reported pose, done in 30 seconds before matches.

## Timing and latency compensation

- **Every measurement gets a capture timestamp:**
  - VisionPortal: `captureTimeNanos`.
  - Limelight: now minus (capture + targeting latency), and minus the staleness of the poll.
- **Pose history ring buffer.** Store the last ~0.5 s of odometry poses in a preallocated array, indexed by time.
- **Apply the correction where it belongs:** compare the vision measurement with the odometry pose *at capture
  time*. That difference is the correction, which you then apply to the current pose.
- **A turret-mounted camera** also needs the turret angle at capture time, so keep the turret angle in the same
  history buffer.

```java
//preallocated, written once per loop
private final double[] historyTime = new double[64], historyX = new double[64], historyY = new double[64], historyHeading = new double[64];
private int historyHead;

public void record(double time, double x, double y, double heading) {

    historyHead = (historyHead + 1) % historyTime.length;
    historyTime[historyHead] = time;
    historyX[historyHead] = x;
    historyY[historyHead] = y;
    historyHeading[historyHead] = heading;
}
```

## Filtering and fusion

- **Gate before you fuse.** Reject a measurement when:
  - it's off the field;
  - it comes from too few tags, too far away;
  - the robot spins faster than your limit;
  - it's stale;
  - its innovation (vision minus predicted) is too large for its expected noise.

  A **Mahalanobis gate** does the last check properly: with 2-D position, reject above **5.99** (95%) or **9.21**
  (99%), which are the chi-square values for 2 degrees of freedom.
- **Weight by quality.** Measurement noise grows with distance, roughly as distance², and shrinks with tag count.
  Feed that into the fusion gain, so a close two-tag pose pulls hard and a far single-tag pose barely moves the
  estimate.
- **A complementary filter is often enough:** `pose += k · (vision − pose_at_capture)` with a small `k` (0.05–0.2
  per accepted frame). Use a Kalman filter when you want the gain to follow the noise automatically.
- **Heading:** let the Pinpoint or IMU own heading, which is what MegaTag2 assumes. Vision heading is usually worse
  than a good gyro over short times.
- **Robust statistics** beat averages for detections: a median of the last 5 centroids, or a trimmed mean, ignores
  a single bad frame.
- **Watch for planar-tag ambiguity.** A single small tag far away can give two pose solutions that flip between
  frames. Prefer multi-tag solves, MegaTag2, or reject single-tag poses beyond a set range.

## Validation: proving it's accurate

1. **Ground truth:** tape-measured field positions, or the robot driven to surveyed poses. Compare vision with them
   *and* with odometry.
2. **Sweep:** distance (near → far), bearing (centre → edge), robot speed (still → full), lighting (dim → bright).
   Record the median and p95 error for each cell. That table *is* your accuracy spec.
3. **Regression set:** keep recorded frames and logs from past sessions, and re-run every pipeline change against
   them before deploying. A threshold change that fixes one venue often breaks another.
4. **Log everything a replay needs:** frame timestamps, raw results, the gate decision, the fused pose. Then a bad
   match can be replayed offline. Write logs on one low-priority thread (see the repo rules).
5. **Run the A/B test that matters:** the same auto, 10 runs with vision fusion and 10 without, comparing where the
   robot ends up. If vision doesn't win clearly, it isn't helping yet.

---

# Part 5: Camera systems

## Camera characteristics that matter

| Characteristic | Options | What it means for us |
|---|---|---|
| **Shutter** | Rolling vs **global** | Rolling captures rows one after another, so fast motion skews the image. Global captures the whole frame at once, which suits a spinning robot and AprilTag pose. |
| **Sensor colour** | Colour vs **mono** | Mono collects more light (no colour filter), so it gets shorter exposure and less blur. It's fine for AprilTags, useless for colour-coded pieces. |
| **Resolution** | 320×240 to 1080p+ | More pixels give more range and precision, but cost USB bandwidth, processing time, and CPU on the hub. |
| **Frame rate** | 30 / 60 / 90 / 120 fps | Sets how fresh the newest frame is, and how many samples the filters get. |
| **Field of view** | Narrow (~55° diagonal) vs wide (~80°+) | Wide sees more of the field but gets fewer pixels per degree and more distortion. Narrow gets more range and precision but loses targets when turning. |
| **Focus** | Fixed vs autofocus | Autofocus hunts and changes the intrinsics. Lock it, or pick fixed focus. |
| **Exposure control** | Auto vs manual (UVC) | Manual exposure and gain are required for low blur and stable colours. Check the camera exposes them. |
| **Stream format** | Uncompressed (YUY2) vs **MJPEG** | YUY2 costs no decompression but lots of bandwidth. MJPEG fits USB 2.0 at high resolution but costs CPU to decompress on the hub. |
| **Interface** | USB UVC, I2C/UART smart camera, USB "Ethernet" (Limelight) | Decides where the compute happens and how results reach the loop. |

**USB bandwidth, worked out.** YUY2 uses 2 bytes per pixel:

- 640×480 at 30 fps = **18.4 MB/s**. Fine.
- 1280×720 at 30 fps = **55 MB/s**. More than USB 2.0 really delivers (around 35–40 MB/s), so frames drop or the
  camera quietly lowers its fps. Use MJPEG or a lower resolution.

## Cameras worth knowing

| Camera | Type | Notes |
|---|---|---|
| **Limelight 3A** | Smart camera, USB-C as an Ethernet device | Covered in Parts 1 and 2. Compute on the camera. One per Control Hub. Rolling shutter, no LEDs. |
| **Logitech C270** | UVC webcam, 720p30, fixed focus, ~55° diagonal FOV | Cheap and common. The SDK has built-in calibrations. Fixed focus is a plus. Low fps and rolling shutter. |
| **Logitech C920 / C920s** | UVC webcam, 1080p30, autofocus, ~78° diagonal FOV | Sharper and wider, with built-in calibrations. **Lock autofocus.** Use MJPEG for high resolutions. |
| **Microsoft LifeCam HD-3000** | UVC webcam, 720p, ~68.5° diagonal FOV | An older FTC staple, still fine for AprilTags. |
| **Arducam USB global-shutter modules** (OV9281 mono, OV9782 colour, 1280×800) | UVC webcam, global shutter, high fps | Popular for FRC AprilTag localization. Good for a spinning robot. You must calibrate yourself. Check the FTC game manual's camera rules before using one. |
| **DFRobot HuskyLens** | I2C/UART AI camera with its own processor | The SDK has a driver. Built-in modes include object tracking and recognition, colour, tags and lines. Low resolution, but no hub CPU. Every read is an I2C transaction on the hub, so it costs loop time like any sensor (see HardwareMonitor). |
| **Pixy2** | I2C/SPI/UART colour-signature camera | Older. Fast colour blobs on the camera. Superseded by the Limelight for most teams. |

Limelight's FRC models (Limelight 3/3G/4) connect over Ethernet and are built for the FRC control system. For
FTC, the 3A is the Limelight to use. Always check the current game manual for camera legality and power rules.

## Not everything should be a camera

The camera handles **far and fuzzy**; simple sensors handle **near and certain**:

- **"Is a piece in the intake?"** A REV Color Sensor V3 (colour + proximity at a few cm), a distance sensor or a
  beam break. It's faster, needs no lighting, and is far more reliable than a camera at that range.
- **"Which colour is the piece we hold?"** A colour sensor inside the robot, shielded from field light.
- **"Where is the robot?"** Odometry (Pinpoint) owns it. Vision only corrects it.
- **"Where are pieces on the field?" / "Where is the goal from far away?"** That's where cameras earn their place.

## Architectures for one robot

1. **Limelight only.** Goal and tag pose plus colour targeting, by switching pipelines. Zero hub CPU. Only one
   pipeline runs at a time, so localization pauses while you search for pieces.
2. **Limelight + webcam.** The Limelight faces the goal and tags (localization and aiming). A webcam with
   VisionPortal faces the intake (piece finding). Each job has its own camera, which is the most capable setup
   within the one-Limelight limit. The cost is the webcam's CPU on the hub.
3. **Webcam(s) only.** Cheapest. Keep the processors light (decimated AprilTag, small colour ROI) and watch loop
   time.
4. **Smart camera on I2C** (HuskyLens) for a narrow job, alongside any of the above.

## Placement and mounting

- **Pick the job first, then the placement.** A goal or tag camera points where the targets will be during
  shooting. An intake camera points at the floor in front of the intake, pitched down so pieces fill the lower
  half of the frame.
- **Height difference helps range accuracy** (see `∂d/∂θ` in Part 4). Avoid a camera at the same height as its
  target, where `ty` barely changes with distance.
- **Turret mount vs chassis mount:**
  - On the turret, the camera keeps the goal in view at all times, but the extrinsics change with turret angle.
    You need turret-angle history at capture time, and the cable has to survive ±200° of turret travel.
  - On the chassis, the extrinsics are fixed and simpler, but the goal can leave the frame.
- **Keep it out of our own way.** Check the full mechanism range (turret sweep, intake up and down) for blocking
  the view. A view blocked half the time looks like random detection dropouts.
- **Stiffness and vibration.** Mount to the frame with short, stiff brackets, away from the flywheel. Check the
  stream at full flywheel speed: if the image shakes, the pose shakes.
- **Protect the lens.** Use a recessed mount or a shroud against ball hits and stray light. Clean lenses before
  every match.

## Wiring, power and failure handling

- **Control Hub USB:** the Limelight 3A goes on the **USB 3.0** port and a webcam on the **USB 2.0** port. More
  devices mean a USB hub, and hubs plus long cables are where most disconnects come from.
- **Strain relief and ESD:**
  - Secure every cable at both ends.
  - Keep USB cables away from moving parts and the frame edge.
  - Remember that static discharge can drop USB devices (modelled by `sim esd`).
- **Boot time:** cameras take seconds to start after power-up or a brownout. Code must cope with "camera not ready"
  at INIT and with "camera lost" during a match, by falling back to odometry and never blocking the loop.
- **Pre-match checklist:**
  - Stream visible.
  - Correct pipeline index.
  - Exposure set for this venue.
  - Calibration check pose OK.
  - Lens clean.
  - Cables seated.

## Lighting

- **Field lighting varies a lot between venues** and even across one field (shadows from structures, glare from the
  perimeter). The Limelight 3A has no LEDs, so it relies on that lighting.
- **Fix everything automatic:** exposure, gain, white balance and focus. Auto modes hunt between frames and move
  your colour thresholds with them.
- **Save one pipeline variant per lighting class** (bright, normal, dim) and switch in INIT after a 5-second check
  of the stream.
- **Glare** from shiny surfaces looks like white (low saturation), so a saturation floor rejects it. Polarizing
  filters exist for stubborn cases.

---

# Part 6: Limelight Lab (live tuning, and uploading from the repo)

## Can pipelines and SnapScripts be uploaded straight from this codebase?

**Yes.** The Limelight has a REST API on port **5807**, and the FTC SDK wraps part of it in `Limelight3A`:

| What | REST (laptop) | `Limelight3A` (robot) |
|---|---|---|
| Python SnapScript | `POST /upload-python?index=N`, the script as plain text | `uploadPython(String, Integer)` |
| Whole pipeline (type, exposure, gain, thresholds, filters, ...) | `POST /upload-pipeline?index=N`, JSON; overwrites it on disk | `uploadPipeline(String json, Integer)` |
| Change some settings live | `POST /update-pipeline`, JSON; `?flush=1` saves | — |
| Read a pipeline back | `GET /pipeline-atindex?index=N` | — |
| AprilTag field map | `POST /upload-fieldmap` | `uploadFieldmap(LLFieldMap, Integer)` |
| Neural model and labels | `POST /upload-nn`, `/upload-nnlabels` | — |
| Switch, reload, snapshots, Python inputs | `/pipeline-switch`, `/reload-pipeline`, `/capture-snapshot`, `/update-pythoninputs` | `pipelineSwitch`, `reloadPipeline`, `captureSnapshot`, `updatePythonInputs` (max 32 numbers) |

The SDK column was checked against the SDK 12.0.0 jar. The REST column is Limelight's REST/HTTP API page.

So this repo keeps them as source files:

```
TeamCode/src/main/assets/limelight/
  p9_blob.py          SnapScript for pipeline 9 (the p<index>_ prefix is the target pipeline)
  p0_<name>.json      a pipeline's full settings, from `tools/limelight.py pull 0`
```

Being in `assets/`, they're versioned in git, show up in Android Studio, and ship inside the APK. There are two
ways to get them onto the camera:

1. **From the laptop:** `python tools/limelight.py push`. It lists what goes where, asks before overwriting,
   backs up the camera's current pipelines to `tools/out/limelight_backup_<time>/`, uploads pipeline JSON first
   (it can change the pipeline type) and SnapScripts second, then reloads.
2. **From the robot:** deploy the APK from Android Studio, run **Limelight Lab**, and hold **A+B** for 1 s. It
   does the same upload using `Limelight3A`. Uploading doesn't need the laptop to reach the camera.

Neural models (`.tflite`) are binary, so they go through the web UI or `/upload-nn`; the CLI doesn't push them
yet.

## Reaching the Limelight from the laptop

| Setup | Address | Needs |
|---|---|---|
| Limelight plugged into the laptop by USB-C | `limelight.local` | Nothing else. This is Limelight's official setup method. |
| Limelight on the Control Hub (real mount, real field) | `192.168.43.1` | Laptop on the robot's Wi-Fi and the **Limelight Lab** OpMode running (INIT is enough). |

The Control Hub doesn't forward the Limelight's ports by itself: the SDK only talks to it internally. **Limelight
Lab** runs `Systems/Vision/LimelightBridge`, a TCP forwarder like FRC's `PortForwarder`. Hub ports 5800–5809 go to
the same ports on the Limelight, so `http://192.168.43.1:5801` is the full web UI, `:5800` the stream, and `:5807`
the REST API.

The bridge:

- Runs on one low-priority daemon thread blocked in `select()`, with network I/O only.
- Exists only while the Lab OpMode runs, never in a match OpMode.
- Takes the Limelight's address from the SDK, falling back to `VisionConstants.LIMELIGHT_FALLBACK_ADDRESS`.

Its telemetry shows open ports, connections and bytes moved, plus the live result:

- pipeline;
- valid, tx, ty, ta;
- staleness and latency;
- colour, tag and detector counts;
- Python output.

Controls: **dpad up/down** switches pipeline, **X** takes a snapshot, **hold A+B** uploads the assets.

## The CLI (`tools/limelight.py`, standard library only)

```
python tools/limelight.py find                     which address answers
python tools/limelight.py watch                    live: pipeline, valid, tx, ty, ta, latency, fps, python out
python tools/limelight.py get exposure             show settings whose name contains "exposure"
python tools/limelight.py set exposure=900 sensor_gain=20         live only; add --save to keep it
python tools/limelight.py sweep exposure 300 3000 300             valid % and jitter per value, then restores it
python tools/limelight.py record 20                log 20 s to tools/out/*.csv: valid %, tx/ty sd, times lost
python tools/limelight.py inputs 100 120 60 130 255 255 1 1      llrobot for p9_blob.py: HSV bounds, use, debug
python tools/limelight.py pull all | push | switch 9 | snap name | urls
```

Run `get` before `set` or `sweep`: the setting names come from the camera's own pipeline JSON.

## Tuning together with Claude (Claude in Chrome)

With the Claude in Chrome extension connected, Claude can open `http://192.168.43.1:5801` (or
`limelight.local:5801`) in a tab. From there it can see the live stream and the settings, and act on them:

- move sliders (exposure, sensor gain, black level, HSV thresholds, contour filters);
- paste SnapScript changes into the code editor;
- read the numbers with `tools/limelight.py`.

Claude does the clicking and measuring. You move the target. A session goes like this:

1. **Set the scene.** Robot on the field at its real height, Lab OpMode in INIT, laptop on the robot's Wi-Fi.
   Claude runs `find`, opens the web UI and screenshots the stream.
2. **Exposure first.** Claude sweeps exposure and gain with `sweep` while the target sits still, and picks the
   shortest exposure that still detects at 100%. Then you wave the target or spin the robot, and Claude checks for
   blur on the stream.
3. **Thresholds.** Claude asks you to put the target in specific places ("1 ft in front, centred", "4 ft away at
   the left edge", "in the shadow under the goal"), snapshots each one, and adjusts the HSV ranges until all of
   them pass and the empty field gives nothing.
4. **Make it fail on purpose.** Claude asks you to hold up the other alliance's colour, a hand, a shirt, or
   field tape. Anything it finds gets fixed with a stricter filter.
5. **Stability.** `record 20` while you slowly move the target across the frame and away. Claude reads the
   valid %, jitter and "times lost", and tightens or loosens the filters.
6. **SnapScript iterations.** For custom logic, Claude edits `p9_blob.py` here, pushes it (after you say yes),
   and reads the `llpython` output live. It tunes the HSV bounds through `inputs` without re-uploading.
7. **Keep the result.** `pull` the tuned pipelines into `assets/limelight/`, so the tuning is in git and the APK.

Rules for that session:

- Claude only changes the camera's settings, never anything that moves the robot.
- It asks before `push` and before `set --save`, because both overwrite tuned pipelines on the camera.
- The web UI saves changes as you make them, so slider moves in the UI are real changes. `pull` them into the
  repo, or note the old values, before experimenting.

---

## Rules for vision code in this repo

- **Java 8 only** (see AGENTS.md). Format with `python tools/pranavfmt.py`.
- **No allocation per frame or per loop.** `Mat`s, arrays and pose buffers are fields created once.
- The **camera thread is EOCV's own.** Don't start more threads for vision. Pass results to the loop through
  `volatile` fields or one swapped object.
- **No hardware calls from vision callbacks.** Hub transactions are serialized, so do them in the loop.
- Keep thresholds, camera poses and fusion gains in `Constants/` (`@Config` for live tuning), not in OpModes.
- There's no `Monitored*` wrapper for the Limelight yet. If you add one, it goes in `Utility/Hardware`.
- Always have a fallback when vision is missing: aim and drive on odometry. Vision only *corrects*.
