# Hand–eye calibration implementation plan for Franka robot arm (ROS 2, eye‑in‑hand)

## Table of contents

1. Scope and assumptions  
2. Frames, notation, and data formats  
3. Phase 1 – System setup  
## 4. Phase 2 – Data collection  
## 5. Phase 3 – Calibration computation (AX = XB)  
## 6. Phase 4 – Validation  
## 7. Phase 5 – Deployment  
## 8. Good pose sampling and numerical conditioning  
## 9. Using existing toolboxes vs. implementing from scratch  
## 10. Summary of key input and output documents  

---

## 1. Scope and assumptions

**Objective:**  
Calibrate the rigid transform between the Franka end‑effector frame `e` and an eye‑in‑hand RGB‑D camera frame `c` using a standard hand–eye calibration formulation \(AX = XB\) (Tsai–Lenz / Daniilidis style) with OpenCV and ROS 2.

**Assumptions:**

- **Robot:** Franka Emika Panda or FR3  
  - Base frame: `r` (robot base, e.g. `panda_link0` / `fr3_link0`)  
  - End‑effector frame: `e` (flange or tool frame, e.g. `panda_link8` / `fr3_link8`)
- **Camera:** Intel RealSense D435 rigidly mounted on the end‑effector  
  - Camera frame: `c` (e.g. `camera_color_optical_frame`)
- **Target:** Planar calibration target (chessboard or Charuco board) rigidly fixed in the workspace  
  - Target frame: `o` (object/board frame)
- **Software stack (ROS 2):**
  - `franka_ros2` (or equivalent Franka driver)
  - `realsense2_camera` ROS 2 driver
  - OpenCV (with `aruco` if using Charuco)
  - Python or C++ nodes for calibration

**Primary result:**

- Rigid transform \( ^eT_c \) (or `eMc`) from end‑effector frame `e` to camera frame `c`.

---

## 2. Frames, notation, and data formats

### 2.1 Homogeneous transforms

Use \(4 \times 4\) homogeneous transforms:

\[
{}^aT_b =
\begin{bmatrix}
R_{ab} & t_{ab} \\
0\ 0\ 0 & 1
\end{bmatrix}
\]

- \(R_{ab} \in SO(3)\): rotation matrix  
- \(t_{ab} \in \mathbb{R}^3\): translation vector

### 2.2 Required transforms

- **Robot base → end‑effector:** \( {}^rT_e \) (denoted `rMe`)
- **Target → camera / target pose in camera frame:** \( {}^cT_o \) (denoted `cMo`)
- **End‑effector → camera (unknown):** \( {}^eT_c \) (denoted `eMc`)

### 2.3 Data formats

#### Robot poses `rMe`

- **Format:**  
  - Either as \(4 \times 4\) matrices, or as `(translation, quaternion)` tuples.

Example per sample `i`:

```yaml
# data/pose_i_rMe.yaml
pose_id: i
rMe:
  translation: [x, y, z]        # meters, in base frame
  quaternion: [qx, qy, qz, qw]  # end-effector orientation in base frame
  matrix:
    - [r11, r12, r13, tx]
    - [r21, r22, r23, ty]
    - [r31, r32, r33, tz]
    - [0,   0,   0,   1 ]
```

#### Target-to-camera poses `cMo`

`cMo` means \( {}^cT_o \): the target/object pose expressed in the camera frame. This is OpenCV's `target2cam` convention from `solvePnP`/Charuco pose estimation, not \( {}^oT_c \).

```yaml
# data/pose_i_cMo.yaml
pose_id: i
cMo:
  translation: [x, y, z]        # meters, in camera frame
  rotation_vector: [rx, ry, rz] # Rodrigues (OpenCV)
  quaternion: [qx, qy, qz, qw]  # optional, converted from rotation vector
  matrix:
    - [r11, r12, r13, tx]
    - [r21, r22, r23, ty]
    - [r31, r32, r33, tz]
    - [0,   0,   0,   1 ]
```

#### Resulting hand-eye transform `eMc`

```yaml
# eMc.yaml
eMc:
  translation: [x, y, z]        # meters, in end-effector frame
  quaternion: [qx, qy, qz, qw]  # camera orientation in end-effector frame
  matrix:
    - [r11, r12, r13, tx]
    - [r21, r22, r23, ty]
    - [r31, r32, r33, tz]
    - [0,   0,   0,   1 ]
  convention:
    from_frame: e
    to_frame: c
```

## 3. Phase 1 – System setup (ROS 2)

### 3.1 Objectives

Physically and logically configure robot, camera, and target.

Ensure ROS 2 topics and TF frames are available.

Fix the calibration target rigidly in the workspace.

### 3.2 Preconditions

Franka robot installed and controllable via ROS 2.

RealSense D435 mounted rigidly on the end‑effector.

Calibration target printed and measured.

Camera intrinsics known or calibrated.

### 3.3 Detailed steps

Step 1.1 – Define frame naming conventions

Action:

Decide and document exact frame names:

Base frame: panda_link0 / fr3_link0

End‑effector frame: panda_link8 / fr3_link8 (or tool frame)

Camera frame used for PnP/calibration: camera_color_optical_frame

Camera mount frame for ROS deployment, if needed: camera_link or camera_color_frame

Target frame: calib_board

Input:

Robot URDF / SRDF

RealSense URDF or TF tree

Output (document):

frames.md
- r: panda_link0
- e: panda_link8
- c: camera_color_optical_frame
- o: calib_board

Note: use the optical frame for OpenCV PnP and hand-eye calibration. If `realsense2_camera` already publishes `camera_link -> camera_color_optical_frame`, publish only the missing end-effector mount transform, such as `e -> camera_link`, or disable duplicate camera TF publishers.

Step 1.2 – Mount camera and measure approximate transform

Action:

Rigidly mount D435 to the end‑effector.

Measure approximate translation and orientation from e to c.

Input:

Mechanical drawings or manual measurements.

Output:

eMc_approx.yaml
eMc_approx:
  translation: [x, y, z]
  quaternion: [qx, qy, qz, qw]
notes: "Rough tape-measure estimate for sanity checks."

Step 1.3 – Install and verify ROS 2 stacks

Action:

Install franka_ros2 and realsense2_camera.

Launch robot and camera nodes (e.g. ros2 launch franka_ros2 ..., ros2 launch realsense2_camera rs_launch.py).

Verify topics:

/joint_states

/tf and /tf_static

/camera/color/image_raw

/camera/color/camera_info

Input:

ROS 2 workspace, package installation.

Output:

system_check.md
- Nodes: list of running nodes
- Topics: /joint_states, /tf, /tf_static, /camera/color/image_raw, /camera/color/camera_info
- Sample ros2 topic echo commands

Step 1.4 – Camera intrinsic calibration

Action:

If not using factory intrinsics, run ROS 2 camera calibration or OpenCV calibration.

Save intrinsics and distortion coefficients.

Input:

Calibration images or ROS 2 calibration session.

Output:

camera_intrinsics.yaml
camera_matrix:
  data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
dist_coeffs:
  data: [k1, k2, p1, p2, k3]
image_size: [width, height]

Step 1.5 – Fix and document calibration target

Action:

Print chessboard or Charuco board.

Measure square size and marker size precisely.

Rigidly fix board in workspace (no movement during calibration).

Input:

Board PDF/PNG, ruler/caliper.

Output:

board_config.yaml
type: charuco   # or chessboard
squares_x: 5
squares_y: 7
square_length: 0.030   # meters
marker_length: 0.022   # meters
dictionary: DICT_4X4_50

## 4. Phase 2 – Data collection

### 4.1 Objectives

Collect synchronized pairs of:

Robot base→end‑effector poses ( {}^rT_e ) (rMe)

Target-to-camera poses / target pose in camera frame ( {}^cT_o ) (cMo)

Ensure good pose diversity for numerical stability.

### 4.2 Preconditions

System setup completed.

Board fixed and visible from multiple robot poses.

Camera intrinsics known.

### 4.3 Pose sampling guidelines

Number of poses:

Minimum ~10–12; recommended 20–30.

Coverage:

Vary robot orientation (roll, pitch, yaw).

Vary distance and viewing angle to the board.

Visibility:

Board fully visible and well‑lit in each pose.

Avoid extreme grazing angles.

### 4.4 Detailed steps

Step 2.1 – Implement a ROS 2 pose capture node/script

Action:

Write a ROS 2 node (Python or C++) that:

Reads current rMe from TF (tf2_ros in ROS 2).

Captures an image from /camera/color/image_raw.

Detects the board and estimates cMo using OpenCV.

Stores both rMe and cMo for each sample.

Records image timestamp, TF timestamp, raw image filename, robot joint state snapshot, and whether the robot was stationary at capture time.

Input:

camera_intrinsics.yaml

board_config.yaml

TF tree (/tf, /tf_static)

Output:

capture_node.py (or .cpp)
capture_node_usage.md

Step 2.2 – Robot pose acquisition (rMe)

Action:

For each pose i:

Move robot to desired configuration (manual or scripted).

Query transform from r to e using tf2_ros::Buffer or tf2_ros::TransformListener. Prefer the image timestamp for TF lookup; use the latest transform only if the robot is stopped and this is documented in the sample metadata.

Convert to homogeneous matrix and store.

Input:

/tf data, joint states, kinematics.

Output (per pose i):

data/pose_i_rMe.yaml
pose_id: i
rMe:
  translation: [x, y, z]
  quaternion: [qx, qy, qz, qw]

Step 2.3 – Image capture and board detection (cMo)

Action:

At same pose i:

Capture color image from /camera/color/image_raw and store the raw image with its ROS timestamp.

Detect chessboard or Charuco corners using OpenCV.

Estimate pose of board relative to camera:

Chessboard: findChessboardCorners + solvePnP.

Charuco: detectMarkers + interpolateCornersCharuco + estimatePoseCharucoBoard.

For planar targets, use a planar-aware PnP method when available, refine the pose, require enough detected corners, and reject poses with negative depth or obvious board-pose flips.

Convert pose to homogeneous matrix and store.

Input:

Camera image, camera_intrinsics.yaml, board_config.yaml.

Output (per pose i):

data/pose_i_cMo.yaml
pose_id: i
cMo:
  translation: [x, y, z]
  rotation_vector: [rx, ry, rz]

Step 2.4 – Synchronization and indexing

Action:

Ensure rMe and cMo correspond to same physical pose i.

Prefer a single node that captures both simultaneously.

If separate, use image timestamps and TF interpolation/nearest-neighbor matching. Do not silently pair a moving robot pose with a stale or latest-only image timestamp.

Input:

pose_i_rMe.yaml, pose_i_cMo.yaml, timestamps.

Output:

data/paired_poses.yaml
raw_images/*.png
sample_metadata.yaml

Example `data/paired_poses.yaml`:

```yaml
pairs:
  - id: 0
    rMe_file: pose_0_rMe.yaml
    cMo_file: pose_0_cMo.yaml
  - id: 1
    rMe_file: pose_1_rMe.yaml
    cMo_file: pose_1_cMo.yaml
    image_file: image_1.png
    image_stamp: 123.456
    tf_stamp: 123.456
    stationary: true
  # ...
```

Step 2.5 – Data quality checks

Action:

Reject samples where:

Board detection fails or is unstable.

Too few Charuco/chessboard corners are detected for a well-constrained pose.

PnP returns negative depth, an obvious planar pose flip, or an implausible board distance.

Reprojection error > threshold (e.g. > 0.5–1.0 pixels).

Visualize detected corners and pose axes on image.

Reserve several valid samples as held-out validation poses rather than using every sample for calibration.

Input:

Raw images, detection results.

Output:

data_quality_report.md
- Number of valid samples
- Reprojection error stats
- Rejected pose IDs

## 5. Phase 3 – Calibration computation (AX = XB)

### 5.1 Objectives

Compute hand–eye transform ( {}^eT_c ) from collected pose pairs.

Use standard AX = XB solver (Tsai–Lenz, Daniilidis, or OpenCV calibrateHandEye).

### 5.2 Preconditions

Sufficient number of valid pose pairs.

rMe and cMo stored consistently.

### 5.3 Mathematical formulation

For each pose `i`:

- Robot base to end-effector: \( {}^rT_{e,i} \), denoted `rMe_i`; this is OpenCV `gripper2base`.
- Target/object to camera: \( {}^cT_{o,i} \), denoted `cMo_i`; this is OpenCV `target2cam`.

For two poses `i` and `j`, define relative motions:

```text
A_ij = inv(rMe_j) * rMe_i
B_ij = cMo_j * inv(cMo_i)
```

Hand-eye equation:

```text
A_ij * X = X * B_ij
```

where \( X = {}^eT_c \), denoted `eMc`, is the unknown camera pose in the end-effector frame.

OpenCV's `calibrateHandEye` can take the absolute `rMe` and `cMo` poses and internally form the relative motions.

### 5.4 Detailed steps

Step 3.1 – Data loading and conversion

Action:

Implement compute_handeye.py that:

Loads all rMe and cMo samples.

Converts them to rotation matrices and translation vectors.

Arranges them into arrays for solver.

Input:

data/pose_i_rMe.yaml, data/pose_i_cMo.yaml, data/paired_poses.yaml.

Output:

calib_input_debug.npz
# Contains arrays:
# R_gripper2base[i], t_gripper2base[i]
# R_target2cam[i],   t_target2cam[i]

Step 3.2 – Call hand–eye calibration solver (OpenCV)

Action:

Use cv2.calibrateHandEye with:

R_gripper2base, t_gripper2base

R_target2cam, t_target2cam

Choose method: cv2.CALIB_HAND_EYE_TSAI, PARK, DANIILIDIS, etc.

Input:

Arrays from Step 3.1.

Output:

R_cam2gripper, t_cam2gripper
# Directly eMc.yaml when gripper == e and camera frame == c

Step 3.3 – Direction and convention sanity check

Action:

Check solver convention:

OpenCV assumes:

- `R_gripper2base`, `t_gripper2base`: rotation/translation from gripper/end-effector to robot base. Use `rMe` directly.
- `R_target2cam`, `t_target2cam`: rotation/translation from target/object to camera. Use `cMo` directly.
- Return value `R_cam2gripper`, `t_cam2gripper`: camera pose in the gripper/end-effector frame, i.e. `eMc` / \( {}^eT_c \) when gripper == `e` and camera frame == `c`.

Do not invert the OpenCV return before writing `eMc.yaml` under these conventions.

Before using real data, run a synthetic test with known `eMc`, known fixed `rMo`, generated `rMe` samples, and derived `cMo` samples. The solver must recover `eMc` within a small tolerance.

Input:

Solver documentation, frames.md.

Output:

eMc.yaml
eMc:
  translation: [x, y, z]
  quaternion: [qx, qy, qz, qw]
  matrix:
    - [r11, r12, r13, tx]
    - [r21, r22, r23, ty]
    - [r31, r32, r33, tz]
    - [0,   0,   0,   1 ]
convention:
  from_frame: e
  to_frame: c

Step 3.4 – Numerical stability and residuals

Action:

Compute residual errors:

For relative-motion residuals, compare `A_ij * eMc` against `eMc * B_ij` for pose pairs.

For absolute consistency, compute `rMo_est = rMe * eMc * cMo` for each sample and report how tightly the fixed board pose clusters in the robot base frame.

Report:

Rotation error statistics (degrees).

Translation error statistics (mm).

Input:

eMc, all rMe, cMo.

Output:

calibration_report.md
- Mean/median rotation error (deg)
- Mean/median translation error (mm)
- Board-pose consistency statistics from `rMo_est`
- Outlier analysis

## 6. Phase 4 – Validation

### 6.1 Objectives

Empirically verify eMc.

Validate qualitatively (visual overlay) and quantitatively (reprojection / 3D error).

### 6.2 Preconditions

eMc.yaml computed.

Calibration target still fixed.

### 6.3 Detailed steps

Step 4.1 – TF integration for validation (ROS 2)

Action:

Publish eMc as static transform in ROS 2:

Use `static_transform_publisher` in a launch file. Prefer the current ROS 2 flagged form:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x X --y Y --z Z \
  --qx QX --qy QY --qz QZ --qw QW \
  --frame-id e --child-frame-id c
```

If deploying through a RealSense frame tree, avoid duplicate TF edges. For example, publish `e -> camera_link` and let `realsense2_camera` publish `camera_link -> camera_color_optical_frame`, or disable the camera driver's duplicate TF publisher and publish `e -> camera_color_optical_frame` yourself.

Input:

eMc.yaml.

Output:

static_transform_e_to_c.launch.py
# Launch file that starts static TF publisher from e to c

Step 4.2 – Project board corners into camera image

Action:

For held-out validation poses:

Get rMe from TF at the validation image timestamp.

Compute rMc = rMe * eMc.

If rMo (board pose in base) known, compute:

cMo_pred = (rMc)^(-1) * rMo.

Project board 3D points using cMo_pred and camera intrinsics.

Compare projected corners with detected corners.

Input:

eMc, camera_intrinsics.yaml, board geometry, TF tree.

Output:

validation_images/pose_i_overlay.png
validation_metrics.yaml

Step 4.3 – 3D target localization consistency

Action:

Use calibrated transform to:

Detect board in camera.

Transform board pose into robot base frame:

rMo_est = rMe * eMc * cMo.

Compare rMo_est across multiple poses; should be consistent.

Input:

eMc, rMe, cMo.

Output:

board_pose_consistency.md
- Mean and variance of rMo_est across poses

Step 4.4 – Application‑level sanity test

Action:

Define 3D points on board (e.g. corners).

Command robot to move so camera centers each point using calibrated transform.

Check physical alignment.

Input:

eMc, application script.

Output:

application_test_notes.md
- Observed offsets
- Qualitative assessment

## 7. Phase 5 – Deployment

### 7.1 Objectives

Integrate eMc into production ROS 2 stack.

Ensure transform is versioned and documented.

### 7.2 Preconditions

Calibration validated and accepted.

### 7.3 Detailed steps

Step 5.1 – Store calibration in central config

Action:

Create canonical config file for robot–camera pair.

Output:

franka_handeye_config.yaml
robot: franka_panda
camera: realsense_d435
frames:
  base: panda_link0
  ee: panda_link8
  camera: camera_color_optical_frame
eMc:
  translation: [x, y, z]
  quaternion: [qx, qy, qz, qw]
date: 2026-06-16
notes: "Eye-in-hand calibration with Charuco, 24 poses, ROS 2."

Step 5.2 – Integrate into ROS 2 launch / URDF

Action:

Option A: static TF publisher in launch file using franka_handeye_config.yaml.

Option B: fixed joint in camera‑on‑tool URDF/Xacro.

Input:

franka_handeye_config.yaml, robot URDF/Xacro.

Output:

franka_with_camera.urdf.xacro
franka_camera.launch.py

Step 5.3 – Versioning and re‑calibration procedure

Action:

Document:

When re‑calibration is required (camera remount, collision, etc.).

How to re‑run Phases 2–4.

Store previous calibrations with timestamps.

Output:

handeye_maintenance_guide.md
- Recalibration triggers
- Step-by-step recalibration procedure

calibration_runs/run_YYYYMMDD/
# Archive of eMc.yaml, reports, data

## 8. Good pose sampling and numerical conditioning

### 8.1 Key principles

Diverse orientations:

Large variation in end‑effector orientation; avoid similar orientations.

Non‑coplanar motions:

Avoid motions that are nearly pure translations or pure rotations about a single axis.

Distance variation:

Vary camera–board distance (near, mid, far).

Field‑of‑view coverage:

Move board across image (center, corners).

### 8.2 Practical checklist

At least:

3–4 significantly different roll angles.

3–4 significantly different pitch angles.

3–4 significantly different yaw angles.

Avoid:

Poses with board at extreme edge of FOV or partially visible.

Very small baselines between poses.

## 9. Using existing toolboxes vs. implementing from scratch

### 9.1 Existing toolboxes

Examples (ROS 2 or ROS‑adaptable):

Franka‑specific hand–eye tools.

Generic ROS hand–eye packages (e.g. easy_handeye‑style).

ViSP hand–eye calibration app.

Pros:

Reduced implementation effort.

Battle‑tested workflows and visualization.

Built‑in ROS integration.

Cons:

Less control over internals.

Fixed assumptions about frames/targets.

Extra dependencies to maintain.

### 9.2 Implementing from scratch

Pros:

Full control over data formats, solvers, and validation.

Transparency and easier debugging.

Customizable for non‑standard setups.

Cons:

More development time.

Maintenance burden.

Requires good understanding of frames and numerical issues.

### 9.3 Recommended approach

For first deployment:

Use an existing toolbox to get a baseline.

Export eMc and study conventions.

For long‑term systems:

Gradually replace parts with your own implementation, keeping the input/output document structure defined here.

## 10. Summary of key input and output documents

### 10.1 Inputs

System and frames:

frames.md

Robot URDF / SRDF

Camera and board:

camera_intrinsics.yaml

board_config.yaml

Data collection:

data/pose_i_rMe.yaml

data/pose_i_cMo.yaml

data/paired_poses.yaml

raw_images/*.png

sample_metadata.yaml

### 10.2 Outputs

Calibration result:

eMc.yaml

franka_handeye_config.yaml

Reports and validation:

data_quality_report.md

calibration_report.md

synthetic_handeye_test_report.md

board_pose_consistency.md

validation_metrics.yaml

validation_images/*.png

Deployment artifacts (ROS 2):

static_transform_e_to_c.launch.py

franka_with_camera.urdf.xacro

franka_camera.launch.py

handeye_maintenance_guide.md


You can now save this directly as `plan.md` in your project.