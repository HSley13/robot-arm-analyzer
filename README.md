# Humanoid Arm Load Analyzer

**Can this robot hold that weight, in that pose, without overloading a joint, and can it go and pick the object up?**

A humanoid robot on a mobile lift base, simulated in real time. You put objects on the floor and on tables, drag the robot around with the mouse (or **move it with your own arm through the webcam**), and ask it to pick things up. Every joint's load is computed live: by an exact physics engine, or by a **neural network trained to imitate it**.

> **Midterm project, Introduction to Artificial Intelligence**, by **HSley13**

![The robot among furniture and objects](image/01-overview.png)

---

## Contents

- [What it does](#what-it-does)
- [Where the AI is](#where-the-ai-is)
- [How it works](#how-it-works)
- [Using it](#using-it)
- [Webcam control](#webcam-control)
- [Tech stack](#tech-stack)
- [Building and running](#building-and-running)
- [Tests](#tests)
- [Retraining the neural network](#retraining-the-neural-network)
- [Project structure](#project-structure)

---

## What it does

### Picks things up, planned with the physics

Click an object, choose *Pick up*, and the robot works out how to reach it. It turns its waist toward it, lowers the lift, leans at the hip, and solves the arm's joint angles with the hand kept upright. Among all the poses that work, it prefers the one that loads the joints least. Then it approaches, grasps, lifts, carries, and puts the object down wherever you click. If it can't, it says why, for example *"too heavy - Shoulder would need 112 N·m (limit 90)"*.

![Pick and place in four steps](image/03-pick-and-place.png)

While the robot holds an object, that object's mass is the arm's payload, so every gauge shows what the robot is really carrying.

### Shows when a joint is overloaded

Each joint has a motor with a torque limit. The gauges, the joint housings and the limbs turn **green** (under 60 % of the limit), **amber** (60-90 %) or **red** (90 % and above).

![Arm held straight out with 14 kg: every joint overloaded](image/04-overload.png)

### Exact physics vs. a neural network

The same torque computation runs two ways: the exact physics engine, and a small neural network trained on that engine's output. The *Payload capacity* scan sweeps the arm from stretched out to folded, asks both "how many kg can it hold here?", and plots them on the same chart with their measured running times.

![Payload capacity scan, exact engine vs. AI](image/05-payload-scan.png)

The neural network's curve follows the exact one closely and is about **9× faster** (0.57 ms vs. 5.19 ms for 40 poses in the run above).

### Driven with the mouse

Grab the hand and drag it anywhere. The robot plans a whole-body pose for that point and **moves** there at speeds its joints could manage: it turns, crouches and leans, all parts finishing together, with no jumps. You can also grab the torso, the lift column or the round base to lean, lift or turn the waist yourself.

Picking things up takes no menus: **double-click an object** and the robot picks it up, and double-click a spot to put it down there. Or drag the hand near an object yourself: an amber ring shows what is in reach, and **clicking the hand** grabs it. Within 20 cm, the hand moves in the rest of the way first. Click the hand again to let go.

![Dragging the hand down to the ball: waist turns, lift lowers, torso leans](image/06-drag-the-hand.png)

Click an object or a spot to get a menu:

![Click menus](image/08-click-menus.png)

### Controlled with your own arm, through the webcam

Stand in front of the webcam (it turns on with the app) and the robot copies your right arm. Raise your hand and its hand rises; reach toward the screen and it reaches forward; reach low and it crouches and leans to follow; tip your hand up or down, or turn your palm over, and its hand does the same. Close your hand near an object to grab it, and open it to set the object down on whatever is below. Step out of view and the robot holds still. You don't have to aim exactly: the app guesses which object you're reaching for (an amber ring lights up around it), pulls the hand toward it as it gets sure, and finishes the grab when you close your hand close by. A pre-trained vision network tracks your arm and hand, and the robot's own planner still moves the body, so it stays smooth and the load analysis keeps running on whatever it carries. See [Webcam control](#webcam-control).

### A five-finger hand, or a warehouse gripper

The hand has 17 joints and closes its fingers to fit each object's size.

![The hand carrying a bottle](image/02-hand.png)

Add or remove a joint (or press *Warehouse*) and the right arm becomes an industrial arm with as many joints as you like and a two-finger gripper. It grasps from above when it can and from the side otherwise. *Humanoid* brings the human arm back.

![Five-joint warehouse arm holding a can it picked from above](image/07-warehouse-arm.png)

### A scene you can build

Ten kinds of objects: bottle, can, mug, box, book, ball, traffic cone, potted plant, dumbbell and kettlebell (its color band follows its weight). Three kinds of furniture: table, low coffee table and kitchen-height counter. Add as many as you like, drag them around (whatever stands on a table moves with it), change any object's mass.

---

## Where the AI is

This project touches several of the main ideas of an introductory AI course.

### 1. The robot as an intelligent agent

| PEAS            | In this project                                                                                                         |
| --------------- | ----------------------------------------------------------------------------------------------------------------------- |
| **Performance** | Reach the goal (grasp the object, put it where asked) without exceeding any joint's torque limit or its range of motion |
| **Environment** | Floor, tables, coffee tables, counters and objects of different shapes and masses, all placed by the user                   |
| **Actuators**   | Waist (turn), lift (up/down), hip (lean), arm joints, 17 finger joints or a gripper                                     |
| **Sensors**     | Object positions and sizes, the robot's own joint angles, the torque each joint must produce, and a webcam watching the person who controls it |

Each request is a **perceive → plan → act** cycle. The robot reads where the object is, plans a sequence of poses, then executes them with smooth, speed-limited motion. If the plan fails, it reports the reason instead of acting.

### 2. Supervised learning: a neural network that imitates physics

Computing joint torques exactly means walking the whole arm (kinematics, Jacobians, masses) for every pose. A **neural network** can learn the mapping *pose + payload → torques* and answer in one cheap forward pass.

```mermaid
flowchart LR
    CFG["Arm description<br/>(JSON)"] --> GEN["Data generator (C++)<br/>random poses inside joint limits,<br/>payloads 0-40 kg"]
    GEN -->|"labels from the exact<br/>physics engine"| DATA["30,000 training +<br/>3,000 validation samples"]
    DATA --> TRAIN["PyTorch MLP<br/>MSE loss, Adam"]
    TRAIN --> W["Weights (JSON),<br/>normalization folded in"]
    W --> INF["Hand-written C++<br/>forward pass"]
    INF --> APP["AI mode in the app"]
```

| | Static model | Dynamic model |
| --- | --- | --- |
| Input | 3 joint angles + payload (4) | angles, velocities, accelerations + payload (10) |
| Network | 4 → 32 → 32 → 3, ReLU | 10 → 96 → 96 → 3, ReLU |
| Output | 3 joint torques (N·m) | 3 joint torques (N·m) |
| Validation mean abs. error | **1.15 N·m** (≈1.3 % of the shoulder's 90 N·m limit) | **0.86 N·m** |
| Validation max abs. error | 19.9 N·m | 16.6 N·m |

- **Labels come from the project's own physics engine**, called from C++, never re-derived in Python, so the network learns exactly the function it replaces.
- **Training:** full-batch Adam (learning rate 10⁻³, step decay) on mean squared error, with inputs and outputs normalized. The normalization is then folded into the first and last layers, so the C++ side is just three matrix multiplies and two ReLUs, with no ML library at runtime.
- **Generalization has limits, and the app respects them.** The network was trained for one arm. Change a limb's length or mass, or add a joint, and the training data no longer describes the arm. AI mode then switches off and the exact engine takes over, a concrete case of distribution shift.

### 3. Search

- **Payload capacity is a binary search.** It finds the largest payload whose torques stay within every joint's limit, using either the exact engine or the neural network as the evaluator.
- **Grasp planning is generate-and-test search over body poses.** The candidates are a grid of lift heights × hip leans: every 2 cm × 5° for the humanoid arm, every 5 cm × 7.5° for other joint counts, where each candidate costs a numeric IK solve. For each, the waist yaw and the arm's angles are solved, and the pose is checked against the constraints: joint ranges, the floor, torque limits, and the hand's required orientation. The poses that pass are scored by a cost function: mainly the peak joint load (lightest wins), plus small penalties for leaning, for tilting the hand away from its preferred angle, and for moving the lift far. Grasp *options* are searched in order too: the gripper tries from above before from the side.
- **Reaching for a dragged point is a cost-based choice.** The robot prefers the pose that changes its body least (waist turn, lift, lean and arm all weighted). Past its reach, it picks the posture that gets closest *without* swinging to a very different one from one mouse move to the next.

### 4. Optimization

- **Inverse kinematics** (which joint angles put the hand *here*?) minimizes the distance to the target with **damped least squares**, a Gauss-Newton / Levenberg-Marquardt step: Δq = Jᵀ(JJᵀ + λ²I)⁻¹ e. Joints that hit their limit are frozen and the rest re-solve (an active-set method).
- For the humanoid's three-joint arm with the hand's angle fixed, there is also a **closed-form** solution. The planner uses it because it's exact and instant, and falls back to the numeric solver for any other joint count.

### 5. Perception: computer vision for control

Webcam control uses two **pre-trained convolutional networks** from Google's MediaPipe: a pose landmarker (33 body points, in metres) and a hand landmarker (21 points per hand). No training is needed; the project's work is turning what they see into commands (`tools/webcam_teleop/teleop_math.py`):

- **Retargeting:** your wrist's position relative to your shoulder, scaled by *robot arm / your arm length*, becomes the robot's wrist target. Copying your hand's *position* rather than your joint *angles* lets the robot's planner choose how to bend, so the safety checks still apply.
- **Hand tilt:** the direction from your wrist to your middle knuckle, measured in the vertical plane the robot's arm moves in, sets the angle of the robot's hand on its wrist. The planner matches it as closely as the wrist's travel allows, but always reaches the point first.
- **Palm twist:** the line across your knuckles, from pinky to index, shows how far your palm is turned. The robot's hand turns about its own axis to match, from palm down to palm up (±90°). Anything it holds turns with it.
- **Keeping hold of a turned hand:** the hand tracker has to see a palm to *find* a hand, but not to keep *following* one, so it is set to rarely let go of a hand it already has. If the whole frame misses your hand, it looks again in a zoomed-in crop around where the body tracker places your hand (MediaPipe Holistic finds hands the same way). If that misses too, the body tracker's own rough hand points keep steering the tilt and twist, and the grip stays as it was.
- **Steady waist:** the robot's arm bends in one plane, so its waist turns to face your hand. When your hand is close to your body, the direction to it is mostly tracking noise, so inside 12 cm the waist keeps its last heading instead of spinning.
- **Grip:** two measures averaged. One is how much the finger joints bend (about 0.3 rad straight, about 3 rad in a fist), which stays the same whichever way the hand faces. The other is fingertip-to-wrist distance divided by palm length (about 2 open, about 1 in a fist). Grabbing and releasing use two thresholds (0.7 and 0.3), so a half-closed hand doesn't flicker between them. A threshold only counts once three frames in a row are past it, so a single misread frame can't drop what you're holding.
- **Holding still while you grab:** making a fist shifts the tracked wrist a few centimetres, right when the robot's hand should stay around the object, and so does opening it. While your grip is on its way (between 35% and 65% closed) and for a quarter of a second after it grabs or lets go, the robot's hand holds where it is. It never holds longer than 0.8 s, so a hand left half-closed still steers.
- **Filtering:** tracking jitters by millimetres every frame. A **1€ filter** (a low-pass filter whose cutoff rises with speed) steadies it at rest without lagging when you move fast. Forward and back, which comes from the webcam's depth guess and is by far the noisiest direction, is smoothed harder.

### 6. Shared autonomy: guessing what you're reaching for

A webcam judges depth poorly, so aiming the robot's hand within a few centimetres of an object is hard. The app helps the way assistive robot arms do: it keeps a guess of what you want and helps only as much as it is sure (`core/include/robotarm/intent.hpp`, used by `TaskController::guidedReach`).

- **Guessing:** every camera frame, each object collects evidence. Being close counts (−1 per 15 cm), and so does heading for it: up to +5 when the hand moves straight at it, nothing when it moves sideways, and minus when it moves away. A still hand has no heading. A "none of these" option stands beside them, for just moving around. Each score follows its evidence with a memory of 0.3 s, and the beliefs are their softmax. So the guess builds over a few frames, and when you curve toward a different object mid-gesture it switches within a few more.
- **Helping:** the hand target is pulled toward the guessed object's grasp by a strength that fades in smoothly with confidence (from 45% to 80% sure) and closeness (from 30 cm to 8 cm), up to 85% of the way. An upright object's hand is also turned back toward level and thumb up. The pull is computed from *your* hand, not from where it was pulled to, so moving away always wins: the confidence drops and the help fades.
- **Finishing the grab:** closing your hand within 20 cm of an object the guess is sure about plans the real grasp (the same planner as pick-up), moves there and grabs on arrival. Opening your hand on the way cancels it.
- **Showing it:** an amber ring around the guessed object brightens with the confidence. Mouse drags are never assisted. `--no-assist` (or **a** in the webcam preview) turns the help off.
- **Learning from you (next):** every session is [recorded](#webcam-control) with the hand's path, the guesses and what was actually grabbed, the labelled data for fitting these numbers to you, and later a learned model of how you reach.

### 7. Planning and acting

A pick-up is a short plan of sub-goals: **approach → grasp → lift → carry**. A put-down is **move above → lower → release → retreat → home**. Each step is a target pose; the controller interpolates between them with eased motion, closes the hand or gripper at the right moment, and hands the object's mass to the physics as payload. Some steps are planned only when reached (the home pose is computed from wherever the arm actually ended up), which is a small form of replanning.

---

## How it works

```mermaid
flowchart LR
    CAM(["Webcam"]) --> MP["webcam_teleop.py (run by the app)<br/>MediaPipe pose + hand,<br/>retarget · hand tilt / twist ·<br/>grip · 1€ filter"]
    MP -->|"WebSocket, JSON"| BR["SensorBridgeServer"]
    BR --> TC
    U(["Mouse · menus · panels"]) --> UI["Qt Quick 3D / QML interface"]
    UI --> TC["TaskController<br/>scene, body, pick / put-down, dragging,<br/>assist (intent guess)"]
    UI --> AC["ArmController<br/>pose, payload, Exact / AI"]
    UI --> HC["HandController<br/>17-joint hand, gestures"]
    TC --> PLAN["Planner (core/)<br/>grasp search · reach · IK"]
    TC --> AC
    TC --> HC
    AC -->|Exact| PHYS["Physics engine (core/)<br/>kinematics → statics / dynamics"]
    AC -->|AI| NN["Neural network (ml/)<br/>C++ forward pass"]
    PHYS --> RES["Torques · safety · capacity"]
    NN --> RES
    RES --> UI
```

**Physics engine (`core/`, C++17 + Eigen, written from first principles):**

- **Kinematics:** each joint is described with Denavit-Hartenberg parameters; forward kinematics chains the transforms to get every joint's position. The hand is a kinematic *tree*: five finger chains branch off the wrist, each placed in the palm with a URDF-style origin (needed for the off-axis thumb).
- **Statics:** the torque each joint needs to hold the pose, from the weight of every link and the payload, through the arm's Jacobian.
- **Dynamics:** the full Recursive Newton-Euler algorithm, with gravity, inertia and Coriolis terms computed separately (tests check they add up exactly to the total).
- **The body:** turning the waist and raising the lift don't change gravity torques. Leaning at the hip tilts the arm within its own plane, which is the same as rotating gravity in the arm's frame, so the analysis stays exact while the robot crouches and leans.

**The interface (`app/`, Qt 6 Quick 3D):**

- The robot, objects and furniture are built from procedural meshes (rounded shells, tubes) written in C++, and every part is placed from the forward kinematics. Change a limb length and the model follows.
- The C++ controllers are exposed to QML as a QML module (`Humanoid.Backend`, using `QML_ELEMENT` + `QML_SINGLETON`). QML binds to their properties and calls their slots.
- The mouse layer works out what a press grabbed (hand, joint, limb, torso, column, base, object, table or empty space) and turns mouse motion into joint motion through each point's on-screen velocity, so dragging feels right from any camera angle.
- **QML hot reload:** save any `.qml` file and the window reloads in place, keeping the robot's state.

### Robot specification

| Part | Details |
| --- | --- |
| Arm | shoulder / elbow / wrist: 0.30 m upper arm, 0.27 m forearm, 0.09 m hand link; torque limits 90 / 45 / 12 N·m |
| Joint ranges | human-like: the elbow only bends one way, enforced everywhere, including inverse kinematics |
| Hand | 17 joints: wrist roll + deviation, 5 fingers (the thumb on its own opposable base) |
| Body | waist turn ±160°, lift 0.25-0.95 m (hip height), hip lean 0-75° |
| Configs | [`config/humanoid_arm.json`](config/humanoid_arm.json), [`config/humanoid_hand.json`](config/humanoid_hand.json) |

---

## Using it

| Mouse                              | Does                                                    |
| ---------------------------------- | ------------------------------------------------------- |
| Drag the **hand** (cyan circle)    | The whole body follows                                  |
| Drag a **joint** or a **limb**     | Swings that joint                                       |
| Drag the **torso / column / base** (amber handles) | Lean / lift / turn the waist           |
| Drag an **object** or a **table**  | Moves it (a table carries what's on it)                 |
| Double-click an **object**         | Picks it up; double-click a spot to put it down there   |
| Click the **hand**                 | Grabs the object it's near (amber ring), or lets go     |
| Click an **object**                | Pick up · move to x, z · set mass · delete              |
| Click the **floor** or a **table** | Put down here · reach here · add an object or furniture |
| Drag empty space                   | Orbit the camera; right-drag pans; the wheel zooms      |

The side panel switches **Humanoid / Warehouse**, adds or removes joints, and has sliders for the body, a demo payload and hand gestures. Each card in the bottom bar is one joint: its angle, its load gauge, and editable torque limit, length and mass. The bottom-left switches choose **Exact / AI** and **Static / Dynamic**.

---

## Webcam control

Webcam control starts with the app: a **Webcam** panel in the top-right corner of the 3D view shows what the camera sees, with your tracked arm drawn on it, and its switch turns it off and on. While it runs, the camera looks over the robot's shoulder (**Behind view**), so your right is the robot's right on screen and reaching toward the screen is its forward. **Free view** puts your own view back. The panel also has the **Assist** switch.

The tracking itself (MediaPipe) is Python, which the app runs for you. Set it up once:

```bash
cd tools/webcam_teleop
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

The app uses that `.venv` if it's there, and `python3` from the PATH otherwise. `--no-webcam` keeps it off at launch. The script still runs on its own too (`python3 webcam_teleop.py`, with its own preview window), for example on another computer with `--host`.

- **First run:** the two MediaPipe models (a few MB) download into `tools/webcam_teleop/models/`. On macOS, allow your terminal to use the camera when asked. If pip finds no MediaPipe for your Python version, create the venv with an older Python.
- **Using it:** stand back far enough for the camera to see your shoulders and right arm. The preview is a mirror showing your tracked arm, a grip bar and the state of things. Run on its own, the script's window also takes keys: **space** pauses (the robot holds still), **a** turns the assist off and on, and **q** quits.
- **If it mirrors you:** if the robot moves left when you move right, add `--mirror`.
- **Options:** `--no-assist` turns off the app's help reaching for things (see [shared autonomy](#6-shared-autonomy-guessing-what-youre-reaching-for)). `--no-wrist` leaves the robot's wrist tilt and twist alone. `--pose-model lite|full|heavy` trades speed for better depth judgement (default `full`).
- **Without a camera:** `python3 webcam_teleop.py --demo` drives the robot with a scripted arm, to check the connection.

**The protocol** (one JSON object per WebSocket message), so other programs can drive the robot too:

| Message | Effect |
| --- | --- |
| `{"type": "reach", "x": 0.5, "y": 0.9, "z": 0.3}` | Move the hand there (metres, world frame); the body follows smoothly, helped onto the object you seem to be reaching for. An optional `"pitch"` (radians, + up from level) also points the hand that way, and an optional `"roll"` (radians, 0 = thumb up, + palm up) turns it about its own axis |
| `{"type": "grip", "amount": 0.8}` | 0 = open … 1 = closed; closing near an object grabs it, opening sets it down |
| `{"type": "assist", "on": false}` | Turn that help off (`true`: on again; it starts on) |
| `{"type": "preview", "jpeg": "<base64>"}` | A camera picture for the app's Webcam panel |
| `{"type": "hold"}` | Stop where it is |
| `{"type": "ping"}` | Answered with `{"type": "pong"}` |

On connecting, a client first receives `{"type": "hello", "robot": …, "shoulder": [x, y, z], "reach": …, "hand": …}`, which describes the robot so arms can be scaled to it (`hand` is how much of the reach is the hand, past the wrist). App flags: `--no-webcam`, `--bridge-port N`, `--bridge-lan` (accept other computers on the network; there's no authentication, so trusted networks only) and `--no-bridge`. The bridge needs the Qt WebSockets module; without it, the app builds without webcam control.

**Recorded sessions.** Every webcam session is written down, to train the assist on later: one [JSON Lines](https://jsonlines.org) file per session in the app's data folder (on macOS `~/Library/Application Support/Humanoid Arm Load Analyzer/sessions/`). A session runs from the first client connecting to the last one leaving. Each line has `t` (seconds since the start) and a `type`:

| Line | Holds |
| --- | --- |
| `start` | The scene: robot kind, objects (id, type, mass, position, grip height), tables, the time and the format `version` |
| `input` | One command as received (`message`), and what the robot made of it: the body `pose` after it (radians, metres), the assist's guess (`intent`: id, confidence, strength) and what is `held` |
| `objects` | The objects again whenever they change, so every grab and release is there |
| `end` | The session is over |

Only the commands are recorded, no camera images. `--record-dir DIR` records somewhere else, and `--no-record` turns it off.

---

## Tech stack

| Technology | Used for |
| --- | --- |
| **C++17** | physics engine, planner, neural-network inference, application |
| **Eigen** | linear algebra (transforms, Jacobians, least squares) |
| **Qt 6 · Qt Quick 3D · QML** | real-time 3D interface, procedural meshes, image-based lighting |
| **PyTorch · NumPy** | training the neural networks |
| **MediaPipe · OpenCV** | webcam control: pose and hand tracking, camera and preview |
| **Qt WebSockets** | the bridge that lets outside programs (the webcam control) drive the robot |
| **nlohmann/json** | robot configs and network weights |
| **GoogleTest · unittest** | 290 unit and integration tests (C++ and Python) |
| **CMake** | build (with `qt_add_qml_module`) |

---

## Building and running

Requires a C++17 compiler, CMake ≥ 3.21, Qt 6.5+ (Quick, Quick 3D, Quick Controls 2, Quick Dialogs 2), Eigen 3 and nlohmann-json.

```bash
cmake -B build -DCMAKE_PREFIX_PATH=/path/to/Qt    # e.g. /usr/local/Qt-6.10.0
cmake --build build
./build/bin/robotarm_app
```

On macOS with Homebrew this works even if Homebrew also has a different Qt installed, and from folders whose path contains spaces. The build includes workarounds for both.

Optional launch flags, mostly for reproducible scenes and screenshots:

```bash
./build/bin/robotarm_app --pose -35,80,-10 --payload 6                 # arm angles (deg), kg in the hand
./build/bin/robotarm_app --pick 1 --put 0.55,-0.6                      # scripted pick-up and put-down
./build/bin/robotarm_app --industrial 5 --pick 3                       # 5-joint warehouse arm picks the can
./build/bin/robotarm_app --table "counter,-0.55,0.45" --add "plant,-0.55,0.30"
./build/bin/robotarm_app --scan 40 --camera 60,-10,2.4 --screenshot shot.png
```

## Tests

```bash
ctest --test-dir build
```

375 tests in 25 suites cover every physics module (kinematics, kinematic trees, statics, dynamics, trajectories, inverse kinematics, payload capacity, grasp and reach planning, intent guessing, the neural-network surrogates, config parsing). They also cover the application layer, including whole pick-and-place sequences stepped in simulated time with both the hand and the gripper, for every object type, and QML loading the C++ module. The webcam path is covered too: grab and release, the assist (guessing, switching mid-gesture, pulling a near miss onto the object, finishing and cancelling a grab), session recording, starting the webcam program from the app, the bridge (including a real local WebSocket connection), hand tilt and twist, and the Python retargeting, steady waist, hand twist, grip and its gate, holding still while grabbing, filter, zoomed-in hand search and control rules.

## Retraining the neural network

```bash
python3 -m venv python/.venv && source python/.venv/bin/activate
pip install -r python/requirements.txt
./build/bin/generate_training_data config/humanoid_arm.json python/data/humanoid_arm_train.csv 30000 1 0 -9.81 0 40
./build/bin/generate_training_data config/humanoid_arm.json python/data/humanoid_arm_val.csv 3000 2 0 -9.81 0 40
python3 python/train_surrogate.py --arm-name humanoid_arm --num-joints 3 --hidden-dim 32
```

The generator's arguments are: config, output file, number of samples, random seed, gravity vector, maximum payload in kg. `train_dynamic_surrogate.py` and `generate_dynamic_training_data` do the same for the dynamic model.

## Project structure

```
core/       physics engine: kinematics, kinematic tree, statics, dynamics, IK, payload capacity, planner
ml/         neural-network inference (C++) + trained weights
app/        Qt Quick 3D interface, the C++ controllers, procedural meshes (app/geometry/)
config/     the robot's arm and hand
tools/      training-data generators (call the physics engine), lighting generator,
            webcam_teleop/ (webcam control: MediaPipe tracking -> robot commands)
python/     PyTorch training scripts
tests/      GoogleTest suites
image/      screenshots used in this README
archive/    the project's earlier industrial 3-DOF arm (not used by the app)
```

## Author

**HSley13**

## License

Released under the [MIT License](LICENSE).
