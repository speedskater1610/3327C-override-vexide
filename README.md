# 3327C-override-vexide
Our (3327C), vex v5 robotics code in rust using the vexide runtime, for the vex override 2026-27

*Inspired by aubie2 pushback code.*

---

First year using rust, but not vexide. Last year we used Zig, through a custom runtime, but because we had some issues with the jumptable, we deciced to use vexides sdk implementation.

## required tooling

| Tool | Install |
|------|---------|
| Rust **nightly** + `rust-src` | `rustup toolchain install nightly && rustup component add rust-src` |
| ARM cross-compiler | `sudo apt install gcc-arm-none-eabi` (Linux) / Homebrew `arm-none-eabi-gcc` (macOS) |
| `cargo-v5` | `cargo install cargo-v5` |

---

## Building

```bash
# Check everything compiles
cargo build -p robot --target armv7a-vex-v5-vexos-eabi -Z build-std=core,alloc

# Release build (smaller binary)
cargo build -p robot --release --target armv7a-vex-v5-vexos-eabi -Z build-std=core,alloc
```

## Uploading to the brain

Connect your V5 brain or controller via USB, then:

```bash
cargo v5 upload --slot 1
```

## Viewing serial output and logging (we use a lot for testing)

```bash
cargo v5 terminal
```

Log lines look like:
```
00:00:123 [INFO] Calibrating IMU...
00:02:048 [INFO] IMU calibrated in 1.925s.
```

---

## Controls library (`packages/3327C`)

The `controls` crate is a `#![no_std]` library providing:

### `hardware`
- **`calibration`** - async IMU calibration with brain screen + controller LCD feedback
- **`encoder`** - `CustomEncoder<TPR>` wrapping `AdiEncoder`, implementing evian's `RotarySensor`; includes `Amt102V` type alias (8192 TPR)
- **`pneumatics`** - `Solenoid` newtype with `extend()` / `retract()` / `toggle()`

### `motion`

#### `controllers`
| Controller | Description |
|---|---|
| `Pid` | Full PID with anti-windup integral clamp and IIR derivative low-pass filter |
| `BangBang` | Simple on/off controller with dead-band tolerance |
| `MotorFeedforward` | kS + kV x v + kA x a feedforward for DC motors |

#### `basic`
- `tank_drive` - left/right stick -> left/right motor voltages
- `arcade_drive` - throttle/steering -> left/right motor voltages
- `curvature_drive` - constant-curvature "cheesy drive" for high-speed arcs

#### `distance_sensor`
- `UntilDistance` - async `Future` that yields until a V5 distance sensor crosses a threshold (with `DistanceOp::LessThan` / `GreaterThan`)

### `subsystems`

#### Non-lift subsystems
| Subsystem | Description |
|---|---|
| `Intake` | Multi-motor intake with `intake()`, `outtake()`, `stop()`, `set_power()` |
| `Claw` | Pneumatic claw wrapping `Solenoid` with `open()` / `close()` / `toggle()` |

#### Lift framework (`subsystems::lift`)

All lift types share the same API and work with any combination of sensor and
controller via generics.  Heights are always in **inches**.

##### Lift types

| Struct | Geometry model | Best for |
|---|---|---|
| `CascadeLift` | Multi-stage sprocket & chain | High-reach stackers, ring elevators |
| `Dr4bLift` | Double-reverse 4-bar | Precise ring/mogo scoring |
| `TwoBarLift` | Single-pivot arm | Claws, mogo intake, hang arms |
| `SixBarLift` | Parallel 6-bar linkage (Newton-Raphson inverse) | High-travel ball/ring scoring |
| `ContinuousChainLift` | Single-loop vertical chain | Fast indexers, single-stage elevators |
| `ScissorsLift` | Multi-stage crossing arms | Skyrise-style extreme height |
| `RackPinionLift` | Pinion on linear rack | Fast precise linear actuation |
| `BeltSlideLift` | Timing belt over pulleys | Fast lightweight linear slides |

##### Sensor sources

| Type | What it reads |
|---|---|
| `MotorIndex(n)` | Built-in encoder of motor `n` |
| `AveragedMotorSensor` | Average of all motor encoders |
| `PotentiometerSensor::new(port)` | ADI potentiometer |
| `RotationSensor::new(sensor)` | V5 rotation sensor (smart port) |
| `FusedSensor::new(vec![...])` | Average of multiple sensors |

##### Controllers

| Type | Description |
|---|---|
| `LiftPid` | PID with anti-windup, IIR derivative filter, dual settled condition (error + velocity) |
| `LiftBangBang` | On/off with dead-band - no tuning needed |

##### Shared API (every lift type)

```rust
// Construction
let mut lift = CascadeLift::with_averaged_motors(motors, geo, pid, presets);
let mut lift = Dr4bLift::new(motors, geo, rotation_sensor, pid, presets);
let mut lift = TwoBarLift::with_motor_encoder(motors, geo, /*index*/ 0, pid, presets);

// Targeting
lift.go_to_inches(18.0);               // absolute inch target
lift.go_to_preset_name("score_high");  // named preset
lift.go_to_preset_index(2);            // by index
lift.go_to_next_preset();              // cycle up - great for a bumper button
lift.go_to_prev_preset();              // cycle down

// Control loop (call every 10 ms)
lift.update(0.010);

// Status
let h   = lift.current_height_inches(); // Option<f64>
let err = lift.error_inches();           // Option<f64>
let ok  = lift.is_settled();             // bool
let p   = lift.nearest_preset_name();    // Option<&str>

// Manual override
lift.set_manual(0.5);   // 50 % power, bypasses controller
lift.stop();            // cut power
lift.zero_sensor();     // zero encoder/sensor at hard-stop
```

##### Defining presets

```rust
let presets = LiftPresets::new()
    .add("lowered",    0.0)
    .add("intake",     2.5)
    .add("score_low",  12.0)
    .add("score_mid",  20.0)
    .add("score_high", 28.5);
```

##### Geometry reference tables

**VEX sprocket pitch radii** (for `CascadeLift` / `ContinuousChainLift`):

| Teeth | Radius (in) |
|---|---|
| 12t | 0.659 |
| 16t | 0.875 |
| 20t | 1.096 |
| 24t | 1.312 |

**Adding a new lift type:** implement a geometry struct with `degrees_to_inches`
and `inches_to_degrees` in `geometry.rs`, add a `src/subsystems/lift/my_lift.rs`
using the `impl_lift!` macro, then export it from `mod.rs`.

### `auton`
The autonomous framework lets you define routes as a readable chain of named actions:

```rust
let route = Route::new("left-qual")
    .then(Action::intake_for(300))
    .then(Action::drive_forward(400.0, 0.7, 2000))
    .then(Action::turn_to_heading(45.0, 1500))
    .then(Action::drive_forward(300.0, 0.65, 2000))
    .then(Action::intake_for(600).race_with(Action::wait(400)));
```

**All action types:**
- `DriveForward` / `DriveBackward` (distance mm, power 0-1, timeout ms)
- `TurnToHeading` (absolute headingdeg, timeout ms)
- `TurnByAngle` (relativedeg, timeout ms)
- `IntakeFor` / `OuttakeFor` (duration ms)
- `IntakeStart` / `IntakeStop` / `OuttakeStart`
- `LiftTo` (encoder ticks, timeout ms)
- `ClawOpen` / `ClawClose` / `ClawToggle`
- `SolenoidExtend` / `SolenoidRetract` / `SolenoidToggle` (solenoid id)
- `Wait` (duration ms)
- `Sequence(Vec<Action>)` - run a list sequentially
- `.parallel_with(other)` - run two actions; finish when **both** complete
- `.race_with(other)` - run two actions; finish when **either** completes

---

## Adding an autonomous route

1. Create `packages/robot/src/routes/my_route.rs`:
   ```rust
   pub async fn run(robot: &mut Robot) { ... }
   ```
2. Add `pub mod my_route;` in `routes/mod.rs`.
3. Call `my_route::run(robot).await;` from `run_autonomous` (or hook it up to the selector).
