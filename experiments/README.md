# Experiment Descriptions

This folder contains the datasets and results for three experiments used to compare the cascaded PID hardware controller, the Hybrid RL–PID hardware controller, and the reinforcement-learning simulation.

For all experiments, the target ball position is the center of the beam.

```text
Reference position: 0 m
Deadband: ±0.010 m
Trial duration: approximately 30 s
```

The deadband allows the controller to balance the ball within ±10 mm of the center rather than at one exact point. 

## Experiment A — Ball Starts at the Center

The ball is initially placed near the center of the beam before the controller is enabled.

The purpose of this experiment is to evaluate steady balancing performance when the system begins close to equilibrium. It is used to observe position stability, small oscillations, drift, beam-angle activity, controller jitter, and the control effort required to maintain balance.

Data are collected for approximately 30 seconds after enabling the controller. The experiment is repeated 10 times for each control method, and the results are averaged to evaluate consistency. 

## Experiment B — Ball Starts Away from the Center

The ball is initially placed approximately 0.10 m from the center of the beam on either the positive or negative side.

Before the outer controller is enabled, the beam is held approximately horizontal using the inner beam-angle PID loop. The PID or Hybrid RL outer controller is then activated and attempts to move the ball toward the center.

The purpose of this experiment is to evaluate recovery from a large initial position error, transient response, settling time, maximum position error, and control effort during recovery.

Each trial runs for up to approximately 30 seconds. The experiment is repeated 10 times for each controller. The simulation is configured to reproduce a similar initial offset condition. 

## Experiment C — Ball Starts at the Center with Disturbances

The ball initially starts near the center of the beam, as in Experiment A.

During the approximately 30-second trial, external positional disturbances are manually applied to the ball.

```text
First disturbance: approximately 5–10 s
Second disturbance: approximately 15–20 s
```

The second disturbance is applied in the opposite direction from the first.

The purpose of this experiment is to evaluate disturbance rejection, recovery behavior, robustness, settling after external disturbances, and changes in controller effort during recovery.

The experiment is repeated 5 times for each controller, and the results are averaged. Because the disturbances are applied manually, their exact magnitude and timing cannot be reproduced identically between trials. The results should therefore be interpreted statistically rather than as perfectly matched individual tests. 

## Evaluation Metrics

The following metrics are calculated for each experiment:

| Metric         | Description                                                  |
| -------------- | ------------------------------------------------------------ |
| RMSE           | Root mean square ball-position error                         |
| MAE            | Mean absolute ball-position error                            |
| Maximum error  | Largest absolute deviation from the center                   |
| Settling time  | Time required to enter and remain within the ±10 mm deadband |
| Control effort | Accumulated magnitude or variation of the beam-angle command |

Results are reported using the mean and standard deviation across repeated trials.

```text
mean ± standard deviation
```
