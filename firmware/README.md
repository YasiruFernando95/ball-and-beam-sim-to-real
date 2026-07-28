# Firmware and PC Software

This folder contains two matched MCU–Python software pairs. Use files from the same pair together, because the communication format and controller responsibilities are different.

## Folder Structure

```text
firmware/
├── mcu/
│   ├── cascaded pid/
│   └── rl pid hybrid/
└── python/
    ├── cascaded pid data logger/
	├── pybullet simulation data logger/
    └── rl pid hybrid data logger/
```

## Baseline Cascaded PID

Use the following pair:

```text
mcu/cascaded pid/
python/cascaded pid data logger/
```

In this configuration, the STM32 runs both control loops:

* Outer ball-position PID
* Inner beam-angle PID

The Python software is mainly used for:

* Sending reference positions
* Monitoring telemetry
* Recording experimental data
* Plotting system responses

Use this pair for conventional PID testing and as the baseline for comparison with reinforcement learning.

## RL–PID Hybrid

Use the following pair:

```text
mcu/rl pid hybrid/
python/rl pid hybrid data logger/
```

In this configuration:

* The Python RL policy acts as the outer controller.
* The RL policy generates a desired beam-angle setpoint.
* The STM32 runs the inner beam-angle PID controller.
* The STM32 applies motor control, command limits, virtual-wall protection, and communication-timeout safety.

Use this pair for trained-policy evaluation and simulation-to-real experiments.

## PyBullet Simulation

To generate simulation data, use the following folder.

```text
python/rl pid hybrid data logger/
```

## Important

Do not mix the MCU firmware from one configuration with the Python software from the other configuration. Their command and telemetry handling may not be compatible.

Before running either configuration:

1. Flash the matching STM32 project.
2. Run the corresponding Python program.
3. Check the configured serial port.
4. Confirm the beam-angle zero and motor direction.
5. Verify that the beam-angle safety limits are active.

Refer to the README inside each subfolder for configuration-specific instructions.
