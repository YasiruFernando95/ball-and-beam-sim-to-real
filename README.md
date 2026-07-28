# Ball-and-Beam Control System

A mechatronic ball-and-beam platform developed for classical control, reinforcement-learning control, and simulation-to-real experiments.

The system uses an STM32F401CCU6 microcontroller to regulate the beam angle through a geared DC motor. Ball position, ball velocity, beam angle, beam angular velocity, motor speed, and the commanded beam-angle setpoint are transmitted to a computer through USB CDC for monitoring, training, and evaluation.

![Ball-and-beam system](docs/images/system_overview.jpg)

## Project Status

Current status: **Functional experimental platform**

Implemented features:

- [x] Mechanical ball-and-beam platform
- [x] DC motor actuation
- [x] Motor encoder feedback
- [x] MT6816 absolute beam-angle sensing
- [x] Graphite-strip ball-position sensing
- [x] STM32 firmware
- [x] USB CDC telemetry
- [x] Inner PID beam-angle control
- [x] Beam-angle virtual walls and braking
- [x] PC-command timeout protection
- [x] PyBullet/Gymnasium RL simulation environment
- [x] Hardware interface for RL beam-angle commands

## Project Objectives

The project aims to:

- Develop a functional ball-and-beam experimental platform.
- Implement reliable real-time sensing and motor control.
- Train reinforcement-learning agents in a physics-based simulation.
- Transfer trained policies to the physical system.
- Compare classical and reinforcement-learning-based controllers.
- Evaluate tracking accuracy, settling behavior, control effort, robustness, and safety.

## System Overview

The controller changes the angle of a V-groove beam to regulate the position of a steel ball.

| Variable | Description | Unit |
|---|---|---|
| `x` | Ball position measured from the beam center | m |
| `xdot` | Filtered ball velocity | m/s |
| `theta` | Beam angle | degrees |
| `thetadot` | Filtered beam angular velocity | degrees/s |
| `omega` | Filtered geared-motor output speed | degrees/s |
| `theta_sp` | Beam-angle setpoint received from the PC | degrees |
| `u` | Inner PID motor command | normalized `-1` to `+1` |

The firmware executes its main sensing, telemetry, and beam-angle control block every **10 ms**, corresponding to **100 Hz**.

## Hardware

### Main Components

| Component | Model or specification | Purpose |
|---|---|---|
| Microcontroller | STM32F401CCU6 Black Pill | Real-time sensing and control |
| Motor | Cytron IG32E, 430 RPM, 14:1 | Beam actuation |
| Motor driver | Cytron MD10C R3 | Bidirectional motor drive |
| Motor encoder | 98 counts per motor revolution | Motor-speed feedback |
| Beam-angle sensor | MT6816 | Absolute beam-angle measurement |
| Ball-position sensor | Graphite-strip potentiometer | Ball-position measurement |
| Display | 0.96-inch SSD1306 OLED | Local measurements and status |
| Power supply | 12 V, 3 A | System power |
| Ball | 25 mm steel ball | Controlled object |
| Beam | 0.30 m acrylic V-groove | Ball rolling surface |

### Mechanical Specifications

| Parameter | Value |
|---|---:|
| Beam length | 0.30 m |
| Ball diameter | 25 mm |
| Firmware beam-angle limit | -25° to +25° |
| Beam construction | Acrylic V-groove |
| Motor gear ratio | 14:1 |
| Frame material | Aluminium extrusion |

### Firmware Peripheral Assignment

| Function | STM32 peripheral or signal | Purpose |
|---|---|---|
| Motor PWM | TIM3 Channel 1 | Motor command magnitude |
| Motor direction | GPIO PB3 | Motor direction |
| Motor encoder | TIM2 encoder mode | Motor-speed measurement |
| Beam-angle sensor | SPI1 with DMA | MT6816 communication |
| Ball-position sensor | ADC1 Channel 2 with DMA | Position-sensor voltage |
| OLED display | I2C1 with DMA | Local display |
| Control/update timer | TIM10 | Encoder and ADC updates |
| PC communication | USB CDC | Commands and telemetry |

A complete wiring diagram should be placed in:

```text
hardware/schematics/
```

## Firmware Architecture

The firmware uses the following custom modules:

```text
dist_adc
ssd1306
mt6816
encoder_speed
motor_driver
virtual_wall
pid
usb_telemetry
```

The main runtime sequence is:

1. Update the ADC-based ball-position measurement.
2. Read the MT6816 beam angle.
3. Estimate ball and beam velocities using filtered numerical derivatives.
4. Read the latest beam-angle setpoint received through USB CDC.
5. Transmit telemetry to the PC.
6. Apply the ±25° command clamp.
7. Run the inner PID beam-angle controller.
8. Apply the virtual-wall safety function.
9. Drive or brake the motor.

The firmware holds the motor at zero for approximately **5 seconds after startup** before enabling the PID controller.

## Control Architecture

### Inner Beam-Angle Controller

The STM32 runs a PID controller that converts the desired beam angle into a normalized motor command.

```text
Controller: PID
Sample time: 10 ms
Output range: -1.0 to +1.0

Kp = 0.02115
Ki = 0.02104
Kd = 0.0001710
```

The controlled variables are:

```text
Input:     measured beam angle, degrees
Setpoint:  commanded beam angle, degrees
Output:    normalized motor command
```

### Outer Controller

The outer controller runs on the PC and sends a desired beam-angle setpoint to the STM32.

The uploaded firmware supports an RL or other external controller through:

```text
g_thsp_cmd
```

The setpoint is limited to:

```text
-25° <= theta_sp <= +25°
```

The exact incoming USB command syntax is implemented by `USB_Command_Process()` in the USB telemetry module and should be documented after that module is added to the repository.

## Reinforcement-Learning Environment

The project includes a custom Gymnasium-compatible PyBullet environment:

```text
rl/training/ball_beam_env_v2.py
```

The environment class is:

```python
BallBeamCleanEnv
```

It is designed for simulation-to-real training and includes:

- PyBullet rigid-body dynamics
- A 0.30 m beam and 25 mm ball
- Beam end stops
- Measured actuator command delay
- First-order actuator lag
- Beam torque and speed limits
- Setpoint-rate limiting
- End-region recovery behavior
- Centering, smoothness, and safety rewards
- Termination for excessive beam angle
- Truncation for persistent end contact or becoming stuck near an end

The environment follows the Gymnasium interface:

```python
observation, info = env.reset()
observation, reward, terminated, truncated, info = env.step(action)
```

### Simulation Timing

| Parameter | Default |
|---|---:|
| PyBullet physics step | `1/240 s` |
| RL control frequency | `100 Hz` |
| Beam-angle limit | `±25°` |
| Normal initial ball range | `±0.06 m` |
| Initial beam-angle range | `±2°` |

The environment automatically calculates the number of PyBullet physics steps per RL control step.

### Observation Space

By default, the environment returns a six-element observation in raw SI units:

```python
observation = [
    x,
    xdot,
    theta,
    thdot,
    theta_sp_delayed,
    theta_cmd,
]
```

| Observation | Unit | Description |
|---|---|---|
| `x` | m | Ball position along the beam |
| `xdot` | m/s | Ball velocity relative to the rotating beam |
| `theta` | rad | Beam angle |
| `thdot` | rad/s | Beam angular velocity |
| `theta_sp_delayed` | rad | Setpoint after the delay buffer |
| `theta_cmd` | rad | Internal actuator command after lag |

The approximate observation limits are:

| Observation | Range |
|---|---:|
| Ball position | `-x_limit` to `+x_limit` |
| Ball velocity | `-3` to `+3 m/s` |
| Beam angle | `-25°` to `+25°` |
| Beam angular velocity | `-50` to `+50 rad/s` |
| Delayed setpoint | `-25°` to `+25°` |
| Filtered actuator command | `-25°` to `+25°` |

A four-state physical observation can be used by disabling the actuator states:

```python
env.include_actuator_state = False
```

This produces:

```python
[x, xdot, theta, thdot]
```

### Action Space

The RL action is a desired beam-angle setpoint in radians:

```python
action = [theta_setpoint]
```

The action space is:

```python
spaces.Box(
    low=[-theta_limit],
    high=[+theta_limit],
    dtype=np.float32,
)
```

With the default configuration:

```text
theta_setpoint in [-25°, +25°]
```

The policy therefore controls the desired beam angle rather than directly commanding PWM.

### Actuator Model

The simulated actuator uses:

1. A setpoint-rate limit
2. A control-step delay buffer
3. A first-order lag
4. A torque-limited beam-joint controller
5. A simplified torque-speed relationship

Default actuator parameters:

| Parameter | Default |
|---|---:|
| Command delay | 13 control steps |
| Delay at 100 Hz | approximately 0.13 s |
| First-order lag time constant | 0.070 s |
| Normal setpoint-rate limit | 140°/s |
| End-recovery rate limit | 220°/s |
| Approximate stall torque | 0.11 N·m |
| Approximate no-load speed | 45 rad/s |

The delayed setpoint is filtered using:

```python
alpha = ctrl_dt / (servo_tau_s + ctrl_dt)
theta_cmd += alpha * (theta_sp_delayed - theta_cmd)
```

### Reset Distribution

At reset, the environment randomizes the initial beam angle and ball position.

Most episodes begin near the center. By default, approximately 25% of resets may place the ball near one of the ends to train recovery behavior.

The delay buffer and actuator state are initialized to the starting beam angle to reduce unrealistic first-step transients.

### Reward Function

The total reward is formed from:

```python
reward = (
    r_center
    + r_servo
    + r_velocity
    + r_progress
    + r_endzone
    + r_position
    + r_offset
)
```

#### Center Reward

A Gaussian reward encourages the ball to remain close to the beam center:

```python
r_center = exp(-0.5 * (x / sigma) ** 2)
```

Default width:

```text
sigma = 0.020 m
```

#### Position Penalty

A normalized quadratic penalty prevents stable off-center behavior:

```python
r_position = -w_x * (abs(x) / x_limit) ** 2
```

Default:

```text
w_x = 0.35
```

#### Progress Reward

The agent is rewarded for decreasing the absolute ball-position error:

```python
progress = previous_abs_x - current_abs_x
r_progress = w_p * progress / ctrl_dt
```

Default:

```text
w_p = 0.25
```

#### Servo-Movement Penalty

Rapid actuator movement is penalized, particularly near the center:

```python
r_servo = -w_u * center_gate * normalized_command_rate**2
```

Default:

```text
w_u = 0.10
```

#### Near-Center Velocity Penalty

A small ball-velocity penalty encourages true settling rather than repeated center crossings.

Default:

```text
w_v = 0.02
velocity scale = 0.25 m/s
```

#### End-Zone Penalty

A quadratic penalty discourages the agent from remaining near either end.

Default:

```text
w_end = 0.35
end margin = 0.020 m
```

#### Success Bonus

The environment defines successful stabilization as remaining within:

```text
|x| < 0.010 m
```

for:

```text
2.0 s
```

At 100 Hz, this requires approximately 200 consecutive control steps.

The first successful hold receives a bonus of:

```text
10.0
```

### Termination and Truncation

#### Beam-Angle Violation

The episode terminates when:

```text
|theta| > 25°
```

Penalty:

```text
-10
```

Reason:

```text
angle_violation
```

#### Persistent End-Stop Contact

Each end-stop contact receives a penalty of `-5`.

The episode is truncated after:

```text
200 consecutive contact steps
```

At 100 Hz, this is approximately 2 seconds.

Reason:

```text
end_contact
```

#### Ball Stuck Near an End

A stuck counter increases when the ball is in the end zone and:

```text
|xdot| < 0.012 m/s
```

The episode is truncated after:

```text
300 consecutive stuck steps
```

At 100 Hz, this is approximately 3 seconds.

Reason:

```text
end_stuck
```

#### Episode Time Limit

The environment does not currently implement a fixed maximum duration internally. Apply Gymnasium's `TimeLimit` wrapper:

```python
env = gym.wrappers.TimeLimit(
    BallBeamCleanEnv(),
    max_episode_steps=2000,
)
```

At 100 Hz, 2000 steps correspond to approximately 20 seconds.

### Episode Metrics

The `info` dictionary includes:

```text
x
xdot
theta
thdot
theta_sp
theta_sp_delayed
theta_cmd
hit_endstop
angle_violation
done_reason
```

At the end of an episode, it also includes:

```text
metrics/success
metrics/settling_time_s
metrics/max_overshoot_m
metrics/band_m
metrics/hold_s
```

`metrics/max_overshoot_m` currently represents the maximum absolute ball-position excursion during the episode.

### Example Environment Usage

```python
import gymnasium as gym
from ball_beam_pybullet_env import BallBeamCleanEnv

env = BallBeamCleanEnv(
    render_mode="human",
    ctrl_hz=100.0,
    theta_limit_deg=25.0,
)

env = gym.wrappers.TimeLimit(
    env,
    max_episode_steps=2000,
)

observation, info = env.reset(seed=42)
terminated = False
truncated = False

while not terminated and not truncated:
    action = env.action_space.sample()
    observation, reward, terminated, truncated, info = env.step(action)

print(info)
env.close()
```

## Repository Structure

```text
ball-and-beam/
├── README.md
├── LICENSE
├── .gitignore
├── firmware/
│   ├── mcu/
│	│	├── cascaded pid/
│	│	└── rl pid hybrid/
│   ├── python/
│	│	├── cascaded pid data logger/
│	│	├── pybullet simulation data logger/
│	│	└── rl pid hybrid data logger/
│	└── README.md
├── hardware/
│   ├── prototype/
│	│	├── cad/
│	│	├── diagrams/
│   └── schematics/
├── rl/
│   ├── training/
│   ├── system id/
│   ├── models/
│   ├── logs/
│   └── README.md
└── experiments/
	├── experiment_a/
	├── experiment_b/
	├── experiment_c/
	├── results_experiment_a/
	├── results_experiment_b/
	├── results_experiment_c/
	├── plot_results.py
    └── README.md

```


## Installation

### Firmware Requirements

- STM32CubeIDE: `[1.18.1]`
- STM32F4 HAL drivers
- USB Device middleware
- SSD1306 display library
- Project-specific sensor, motor, PID, and telemetry modules

Read the README.md file in the firmware folder before continuing

Build and flash the relelvant firmware to the STM32F401CCU6.

### Python Requirements

Recommended Python version:

```text
Python 3.10 or 3.11
```

Create and activate a virtual environment:

```bash
python -m venv .venv
```

Windows:

```bash
.venv\Scripts\activate
```

Linux or macOS:

```bash
source .venv/bin/activate
```

Install the required packages:

```text
numpy
pandas
matplotlib
pyserial
gymnasium
pybullet
stable-baselines3
torch
```

Remove packages that are not used by the final repository.

## Running the Hardware

1. Inspect the beam, ball, sensor strip, and motor mechanism.
2. Place the beam near its neutral position.
3. Connect the STM32 through USB.
4. Connect the 12 V motor supply.
5. Confirm that the MT6816 angle is reasonable.
6. Confirm that the virtual-wall limits are ±25°.
7. Start the PC controller and begin sending beam-angle commands.
8. Stop operation immediately if the measured angle or motor direction is incorrect.

The firmware waits approximately 5 seconds before enabling motor control.

## Communication Protocol

### STM32-to-PC Telemetry

The STM32 sends one newline-terminated ASCII telemetry record every 10 ms:

```text
X:<x_m>,XD:<xdot_m_s>,TH:<theta_deg>,THD:<thetadot_deg_s>,W:<motor_speed_deg_s>,ANG:<setpoint_deg>
```

Example:

```text
X:-0.0342,XD:0.1285,TH:3.417,THD:-5.231,W:42.580,ANG:4.000
```

| Field | Unit | Description |
|---|---|---|
| `X` | m | Ball position relative to the center |
| `XD` | m/s | Filtered ball velocity |
| `TH` | degrees | Measured beam angle |
| `THD` | degrees/s | Filtered beam angular velocity |
| `W` | degrees/s | Filtered geared-motor output speed |
| `ANG` | degrees | Latest beam-angle setpoint received from the PC |

The transmitted values are generated from the current firmware state. `ANG` reports the latest received command variable; the motor-side setpoint is subsequently forced to zero if the command timeout expires.

### PC-to-STM32 Commands

Incoming USB CDC data are passed to:

```c
USB_Command_Process(buf, len);
```

The command updates the global beam-angle setpoint used by the hardware controller.

The exact command-string syntax is defined in the `usb_telemetry` source files and should be documented here after those files are added.

### Command Timeout

If no valid command update is received for more than:

```text
100 ms
```

the firmware changes the active beam-angle setpoint to:

```text
0°
```

## Calibration and Firmware Constants

### MT6816 Beam-Angle Zero

Current zero count:

```text
8344
```

The raw 14-bit encoder value is wrapped around zero and converted using:

```text
360 / 16384 degrees per count
```

This value is hardware-specific.

### Ball-Position Sensor

Current ADC configuration:

```text
Reference voltage:       3.3 V
Resolution:              12 bit
Low calibration voltage: 0.64 V
Mid calibration voltage: 1.84 V
High calibration voltage:2.97 V
Inversion:               disabled
Gamma:                   1.0
Filter alpha:            0.1
DMA buffer length:       64 samples
```

The normalized distance is converted to a centered coordinate using:

```c
x_norm = 2 * distance - 1;
x_m = -0.1375 * x_norm;
```

The resulting nominal position range is approximately:

```text
-0.1375 m to +0.1375 m
```

The sign convention is defined by the negative scale factor.

Repeated use of the graphite and copper strips will degrade the position sensor. Therefore, it is advisable to recalibrate the position sensor after significant usage. In case of unusual readings, it is recommended to replace the graphite and copper strips.

### Velocity Estimation

Ball and beam velocities use a numerical derivative followed by a first-order low-pass update:

```c
filtered += DERIV_ALPHA * (raw_derivative - filtered);
```

Current values:

```text
DT = 0.01 s
DERIV_ALPHA = 0.25
```

### Motor Encoder

```text
Counts per motor revolution: 98
Gear ratio:                  14:1
Output-speed LPF cutoff:     2 Hz
```

### Beam-Angle PID

```text
Kp = 0.02115
Ki = 0.02104
Kd = 0.0001710
Sample time = 10 ms
Output limits = -1 to +1
```

## Safety Features

The uploaded firmware includes:

- Beam-angle command clamping to ±25°
- Virtual-wall protection based on measured beam angle
- PID integrator reset when the virtual wall blocks motion
- Speed-proportional braking near a blocked limit
- Motor command saturation
- A 100 ms PC-command timeout
- A 5-second startup motor-disable period
- Zero motor output while the inner controller is disabled

The virtual-wall braking command is based on:

```c
brake = -0.2 * motor_speed_rad_s;
```

and is limited to:

```text
-1.0 to +1.0
```

Do not operate the system until the beam-angle sign, motor direction, encoder zero, and virtual-wall response have been checked.

In addition, phyical side bars have been installed on the setup to prevent the beam from continuous rotation. Continuous rotation may tangle the absolute encoder cable and damage electrical connections


## Known Limitations

- The graphite-strip ball-position sensor may introduce noise and nonlinearities.
- Ball and beam velocities are estimated using filtered numerical derivatives.
- The actuator model is a simplified delay, lag, and torque-speed approximation.
- Friction, ball mass, delay, and sensor noise are not yet randomized.
- The environment uses optional end-recovery delay relief that reduces strict actuator realism.
- The RL target is fixed at the beam center.
- The environment does not implement its own maximum episode duration.
- The firmware telemetry does not include the inner PID output.

## Planned Improvements

- Improve position sensing mechanism to a robust solution less susceptible to wear, degradation and long term drift.
- Investigate hardware aware fine tuning
- Compare multiple RL algorithms on the same platform.
- Apply full RL-based control by allowing policy to output direct motor commands
- Add configurable target trajectories.


## Citation

```bibtex
@misc{fernando_ball_beam_2026,
  author    = {Yasiru Suharshana Fernando},
  title     = {Ball-and-Beam Control System},
  year      = {2026},
  publisher = {GitHub},
  url       = {[repository URL]}
}
```

## License

This project is licensed under the **[choose a license]**.

See the `LICENSE` file for details.

## Authors

**Yasiru Suharshana Fernando**

PhD Candidate in Mechatronics and Machine Intelligence
Faculty of Advanced Science and Technology  
Asian Institute of Technology  

- Email: `[st125391@ait.asia, yasiruf@yahoo.co.uk]`
- ORCID: `[https://orcid.org/0009-0000-1786-4137]`
- Website: `[https://yasirufernando.com/]`
- LinkedIn: `[https://www.linkedin.com/in/yasirufernando/]`

**Narong Aphiratsakun**

Dean of the Faculty of Engineering, Science and Technology 
Assumption University, Bangkok

- Email: `[narongphr@au.edu]`
- ORCID: `[https://orcid.org/0009-0009-0956-7656]`

**Xavier Jonathon Blake**

PhD Candidate in Mechatronics and Machine Intelligence
Faculty of Advanced Science and Technology  
Asian Institute of Technology  

- Email: `[xavierjblake@gmail.com]`
- ORCID: `[https://orcid.org/0009-0008-8639-558X]`

## Acknowledgements

The authors would like to express their sincere gratitude to Assumption University of Thailand and the Asian Institute of Technology for their support and institutional assistance throughout this research.