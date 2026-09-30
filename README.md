# JetRover Control Experiments

**Predict first, then measure.** A self-funded series of experiments on a mobile robot with a six-axis arm, about one question I keep running into: *why does a real machine behave differently from the numbers I calculated for it, and what does it take to make it keep working when conditions turn against it?*

Yu Chen (Roger) · Grade 12 · Suzhou, China

> 📷 **[Add photo]** The JetRover on the floor of my room, arm extended. This is the first thing a reader should see.

---

## Why this exists

During my internship at HIKROBOT I watched automated guided vehicles drift off their planned paths on dusty, worn concrete. It was the same drift I had blamed on my own code all season in VEX robotics. I wanted to study it properly, but there was no spare hardware for an intern to experiment on.

So I bought a robot with my own savings: a Hiwonder JetRover with a Mecanum-wheel base, a six-axis arm and ROS 2. Nobody assigned these experiments. Each one exists because the previous one left me with a question I could not answer.

## The route at a glance

| Stage | The question I was asking | What I found |
|---|---|---|
| A. ESP32 bench | How does a PID loop actually behave, before I trust one on real hardware? | No perfect gains, only trade-offs. Integral windup was real and a clamp fixed it. |
| 1–2. Setup and mapping | What is actually inside this robot, and how do I talk to it? | A confirmed hardware map, and one of my own assumptions corrected. |
| 3. Step response vs load | Can I predict how the arm's built-in controller reacts to a heavier load? | **Overshoot grew with load (6.0° → 8.4° → 10.6°), exactly as I predicted from gravity.** |
| 4a. Heating under load | Can heat give me an independent check on the load the servo is carrying? | 400 g heated the shoulder about 2.7× faster than no load. |
| 4b. Maximum acceleration | Does the base joint reach the acceleration I calculated by hand? | **No. It reached well under half, and the real limit was a speed cap in the firmware.** |
| 5. Heading under a friction disturbance | Can my own controller keep the whole robot straight when one side slips? | **Closed loop cut RMS heading error to about a quarter of the open-loop run on the same surface.** Then I found where it stops working. |

## How I worked

- **Prediction before measurement.** For every main experiment I wrote down what I expected and why before I ran it. When the result disagreed, the disagreement was the interesting part.
- **Raw data kept.** Every plot comes from CSV logs in [`data/`](data/). Nothing is hand-copied.
- **Honest limits.** Each section ends with what the experiment cannot tell me.
- **Help I used.** I learned much of the theory with AI assistants (ChatGPT and Claude) as tutors. They explained concepts I had not met yet, helped me write the ROS 2 logging nodes and helped turn my CSV logs into plots. The hardware work, the choice of each experiment, the predictions, every measurement and the conclusions are mine. **[Roger: edit this so it matches exactly how you worked.]**

---

## Part A: Learning the loop on an ESP32

Before trusting a controller on a real robot, I wanted to see a PID loop misbehave somewhere cheap.

### A1. Toolchain and a first blink

Getting an ESP32 to compile and flash took longer than the blink itself. The Arduino toolchain failed with a storage path error until I moved its data and build-cache directories.

![ESP32 blink setup](figures/a1_esp32_blink_setup.jpg)

### A2. PID on a simulated plant

I wrote a PID controller from scratch on the ESP32 and pointed it at a **simulated** system, a number that should reach 100. The plant is a model running on the chip, not a real motor. The code is in [`code/esp32/pid_simulated_plant.ino`](code/esp32/pid_simulated_plant.ino).

**Version 1 had no inertia.** The output changed the position directly, so the curve just crept towards the target and carried straight on past it with no sign of turning back. A system with no mass does not behave like anything real.

![v1 plant, no inertia](figures/a2_pid_v1_plant_no_inertia.png)

**Version 2 added inertia and damping.** The controller's output now acts like a force: it changes velocity, velocity changes position, and a damping factor lets the system settle. Now the tuning problems became visible:

| Change | What happened | Why |
|---|---|---|
| Ki = 0.02 | Overshot to about 105 and stayed stuck there | **Integral windup.** The integral kept growing while the system was still far from the target, then took forever to unwind. |
| Ki = 0.005 | No overshoot, but stopped at about 101.5 | Too little integral action to remove the last bit of **steady-state error**. |
| Ki = 0.01, timing checked | Better, but still off | I found my loop waited `delay(100)` while the maths assumed `dt = 0.05`. The code was lying to itself about time. |
| Ki = 0.02 **plus an integral clamp** (±50) | Settled to within about 0.3% of the target and kept creeping in | The clamp stops windup, so a useful Ki no longer causes a huge overshoot. |

![Windup at Ki = 0.02](figures/a2_pid_ki002_windup.png)
![Steady-state error at Ki = 0.005](figures/a2_pid_ki0005_steady_state_error.png)
![Final version with anti-windup](figures/a2_pid_antiwindup_final.png)

**What I took from it:** there is no perfect set of gains, only trade-offs. More integral kills steady-state error but invites windup. Less integral is calm but never quite arrives.

### A3. Seeing PWM on a logic analyzer

A PID output is useless until it becomes something a motor can feel. On a microcontroller, that is usually PWM: switching a pin on and off fast, with the duty cycle setting the effective power. I generated a 1 kHz signal alternating between 25% and 75% duty ([`code/esp32/pwm_test.ino`](code/esp32/pwm_test.ino)) and captured it on a logic analyzer in PulseView.

![PWM on the logic analyzer](figures/a3_pwm_logic_analyzer.png)

This completed the chain I wanted to understand: **sensor → error → PID → PWM → motor.**

> 📷 **[Add photo]** The ESP32 wired to the logic analyzer.

---

## Part B: The JetRover

### Stage 1. Environment and remote access

I set up remote access to the robot's onboard computer, built my own ROS 2 workspace and loaded the robot model in RViz so I could see every joint.

![URDF in RViz](figures/s1_rviz_urdf.png)

### Stage 2. Mapping the arm

Before measuring anything I needed to know what I was measuring. I queried every bus servo through the robot's ROS 2 service and built a hardware map:

| ID | Joint | Servo | Range (units) |
|---|---|---|---|
| 1 | Base rotation | HTS-20H (20 kg·cm) | 0–1000 |
| 2 | Shoulder | HTD-35H (35 kg·cm) | 0–1000 |
| 3 | Elbow | HTD-35H (35 kg·cm) | 0–1000 |
| 4 | Wrist pitch | HTD-35H (35 kg·cm) | 0–1000 |
| 5 | Wrist roll | HTS-21H | 0–1000 |
| 10 | Gripper | HX-12H | **0–700** |

One servo unit is 0.24°. Two things mattered later. The gripper has a different range, so sending it 1000 would be a mistake. And the wrist pitch is a 35 kg·cm servo, not the 20 kg·cm I had assumed from my notes. My later torque calculations use the corrected table.

![Reading a servo's state](figures/s2_servo_get_state.png)

---

### Stage 3. How does the arm's controller react to load?

**Why this experiment.** Each bus servo runs its own position controller inside the firmware. I cannot see or change its gains. What I *can* do is poke it and watch. On the ESP32 I had seen PID effects in a simulation. Here I wanted to see whether I could predict them in a real servo carrying a real load.

**My prediction, written before the runs.** A heavier load should rise more slowly, which is the obvious inertia effect. The less obvious prediction was about **overshoot**. The intuitive guess is that a heavier arm is more sluggish and overshoots less. I expected the opposite. During the move the load passes over the vertical, and at that point gravity flips from resisting the motion to pushing it along. Right when the servo needs to brake, a heavier load is pushing harder, so it should overshoot *more*.

**Method.**
- Shoulder servo (ID 2), a fixed step of −60° (−250 servo units), commanded move time 0.3 s.
- Loads of 0 g, 200 g and 400 g, fixed about 20.5 cm from the shoulder axis.
- My own ROS 2 node ([`step_logger.cpp`](code/ros2_stage3_logger/src/step_logger.cpp)) fires the step and logs target and actual angle at 50 Hz for 6 s.

For scale, the load's gravity torque at 20.5 cm is at most about 0.40 N·m for 200 g and 0.80 N·m for 400 g. That is well inside the servo's rating, so any change in behaviour is about control, not strength.

> 📷 **[Add photo]** The load fixed to the arm, with the 20.5 cm distance visible.

**Results.**

![Step response, no load](figures/s3_step_no_load.png)
![Step response vs load](figures/s3_step_vs_load.png)

| Load | Rise time (to −54°) | Overshoot past −60° | Final angle (steady-state error) |
|---|---|---|---|
| 0 g | 0.40 s | **6.0°** (to −66.0°) | −63.1° (−3.1°) |
| 200 g | 0.42 s | **8.4°** (to −68.4°) | −64.4° (−4.4°) |
| 400 g | 0.50 s | **10.6°** (to −70.6°) | −65.8° (−5.8°) |

**What I concluded.** All three trends are monotonic and all three have a physical reason:
1. **Rise time grows with load.** More inertia means slower acceleration.
2. **Overshoot grows with load.** This is the counter-intuitive one, and it matched my prediction. Gravity changes sign as the load passes over the vertical, so the heavier load pushes harder at exactly the moment the servo is trying to stop.
3. **Steady-state error grows with load.** At the final pose, gravity keeps pulling the arm down. The servo's internal controller has finite stiffness, so it settles a little past the target, and further for heavier loads.

This was my favourite moment of the project: a behaviour I had first seen as a number on a simulated plant showed up in a real industrial-style servo, and I had predicted it from mechanics before measuring it.

> 🎥 **[Add video link]** 0 g vs 400 g step, ideally in slow motion.

**Limits.** I could not see the servo's internal gains, so I can explain the trends but not model them exactly. **[Roger: say how many runs per load you did.]**

---

### Stage 4a. Side experiment: does heat agree?

**Why.** Holding a load takes torque, torque takes current, and current makes heat. If that chain holds, heat gives me a second, completely independent way to see the load the servo is carrying.

**Method.** Shoulder servo holding a fixed pose, with 0 g and then 400 g. My node ([`temp_logger.cpp`](code/ros2_stage3_logger/src/temp_logger.cpp)) read its temperature every 10 s. The room was about 31 °C.

![Heating under load](figures/s4_heating_under_load.png)

**Result.** Over 180 s, the servo warmed by **30 °C with 400 g** but only **11 °C with no load**, a ratio of about 2.7.

**What I concluded.** More load means more heating, so the chain holds. If heating goes roughly with current squared, and current with torque, the 2.7 ratio should match the squared ratio of the total shoulder torque (arm plus load) in the two cases. Checking that against my torque calculation is on my list.

**Limits, stated honestly.** Neither run reached a plateau; both were still climbing. The no-load run also started warm (57 °C, against 45 °C for the 400 g run) because the servo had not fully cooled. So this is a qualitative trend, not a precise measurement. It is also why I made the next experiment my main one: it measures the load's effect through motion, which is cleaner than waiting for heat.

---

### Stage 4b. Main experiment: can the base reach the acceleration I calculated?

**Why.** If I know the arm's moment of inertia and the motor's torque, I should be able to predict its maximum angular acceleration. I wanted to know whether a spec sheet and some mechanics are enough to predict a real machine.

**My prediction, calculated by hand.** With the arm stretched out horizontally, rotating about the vertical base axis (ID 1), I modelled every part. Compact parts (motors, gripper) were treated as point masses, I = m·d². Long links (upper arm, forearm) were treated as rods using the parallel-axis theorem, I = (1/12)·m·L² + m·d².

![My moment of inertia worksheet](figures/s4b_inertia_worksheet.jpg)

- Total: **I ≈ 0.0235 kg·m²**
- Base servo stall torque: 20 kg·cm = 1.96 N·m. I used 70% of that, 1.37 N·m, as the usable figure.
- **Prediction: α_max = τ / I ≈ 1.37 / 0.0235 ≈ 58 rad/s².** Using the full stall torque instead gives about 83 rad/s².

**Method.** The base rotates through a fixed angle. I kept shortening the commanded move time, from 0.3 s down to 0.05 s, which demands a steeper and steeper acceleration. The idea was that once I demand more than α_max, the real motion can no longer keep up, and the point where it falls behind tells me the true limit.

**Result.**

![Saturation: every commanded duration gives the same motion](figures/s4b_saturation_curves.png)

The curves for 0.3, 0.2, 0.15 and 0.05 s lie almost exactly on top of each other. I demanded **36 times more acceleration** and the arm moved identically every time, taking about 0.6 s. The angular velocity plot shows why: every run climbs to the **same ceiling of about 220 °/s (≈ 3.9 rad/s)**, holds it, then slows down.

The measured acceleration is **roughly 20–30 rad/s²**, depending on the method. Peak velocity divided by acceleration time gives about 20; a noisy second-difference of the angle data gives 21–31. **Either way it is well under half of my prediction.**

**What I concluded.** My arithmetic was right, but my model of the motor was wrong in three ways:
1. **A speed cap in the servo firmware is the main limit.** Five runs hitting the identical ceiling is the clearest evidence. However hard I ask, the firmware will not let the servo go faster than about 220 °/s.
2. **Stall torque is not the torque you get while moving.** Stall torque is measured with the motor held still. As it speeds up, the available torque falls.
3. **Friction and gearbox losses** take a further share.

The lesson I keep coming back to: **a spec-sheet number describes one specific situation, and it is easy to use it in a situation it does not describe.** It is the same lesson I met on the wall-climbing robots I tested at TRI. A maximum rating is something a machine can *reach*, not something it can be held at, and designing as if it could be held there is exactly where problems start.

**Limits.** In my worksheet the wrist-roll and gripper servo names are swapped compared with the hardware table in Stage 2. **[Roger: check whether their masses were swapped too, and correct the total if so.]** The 0.5 s run started from a different position and never reached the end stop, so it is shown but kept out of the comparison. My acceleration figure is the weakest number in this repo. The servo reports position at about 50 Hz, and in the velocity plot most of the rise happens within roughly 0.1 s, so the true peak acceleration could be higher than 20. Re-deriving it properly from the raw CSV is on my list. The main conclusion does not depend on it: whatever the exact figure, the firmware speed cap is the limit, not my inertia estimate.

> 🎥 **[Add video link]** 0.3 s and 0.05 s commands side by side, looking identical.

---

### Stage 5. Keeping the whole robot straight when one side slips

**Why.** This is the question that started everything: the drift I saw at HIKROBOT. Can I make the whole vehicle hold a straight line when one side has less grip?

**What I found first.** The chassis does not expose raw wheel-encoder counts, only the speeds it has been told to produce. So a per-wheel closed loop was impossible from my level, and from my side the wheels are effectively open-loop. What the robot *does* publish is `/odom`, a pose estimate from an EKF that fuses wheel odometry with the onboard IMU. I tested that it was real by turning the robot by hand and watching both the odometry and the IMU heading follow the actual rotation.

> 📷 **[Add photo]** Turning the robot by hand while watching the heading in the terminal.

So I built what the sensing allowed: **an outer heading loop around the chassis's own speed control.** My node ([`heading_pid.cpp`](code/ros2_stage3_logger/src/heading_pid.cpp)) locks the starting heading as its target, drives forward, and corrects with a turning command. It uses Kp = 1.5, Kd = 0.2 and Ki = 0, running at 50 Hz. Setting Kp to 0 gives the open-loop baseline with nothing correcting it.

**My prediction.** On a normal floor the robot should drive almost straight on its own. With a low-friction strip under one side, that side slips, the robot does not know, and it drifts. My controller should pull the heading back.

**Method.** Forward at 0.1 m/s for 10 s (about 1 m), three conditions: open loop on normal floor, open loop with paper under one side, and closed loop with paper under one side. I first tested the controller with the wheels off the ground.

> 🎥 **[Add video link]** Wheels-off-the-ground test, then the three straight-line runs.

**Results.**

![Heading error vs time](figures/s5_heading_error.png)

| Condition | Final heading error | Max error | RMS error |
|---|---|---|---|
| Open loop, normal floor | −0.82° | 1.09° | 0.78° |
| Open loop, paper under one side | +0.61° | 0.71° | 0.37° |
| **Closed loop, paper under one side** | **+0.04°** | **0.28°** | **0.10°** |

With the same disturbance, closed loop cut the RMS heading error from 0.37° to 0.10°, **about a quarter**. Against the open-loop floor run it is about an eighth, and the final error is almost zero. The green line stays pinned near zero while the other two wander away.

**Then the part that did not fit.** I also measured how far sideways each run ended up, with a tape measure: about 2 cm (open loop, floor), 1.1 cm (open loop, paper) and 0.3–0.5 cm (closed loop, paper). The paper run should have drifted *more* than the bare floor, but it drifted less. The reason is that the disturbance I introduced was about the same size as the floor itself: dust, grit, the seams between floorboards, any slight slope, and the small jolt at start-up all move the robot by a centimetre or two. My signal was buried in the noise of my own bedroom floor. So I trust the heading data, which was logged continuously, and not the tape-measure numbers.

**The boundary I found.** A heading loop controls which way the robot *faces*, not where it *is*. A Mecanum robot can keep a perfect heading and still slide sideways, and my controller cannot see that slide at all. Closing that gap needs a position measurement that does not come from the wheels, most likely a camera.

---

## What I found, in one place

- **Prediction beats hindsight.** The overshoot result in Stage 3 was only convincing because I wrote the gravity argument down before the data existed.
- **A spec number describes a situation, not a machine.** Stall torque describes a motor held still. Rated power describes a motor turning at rated speed. Most of my wrong predictions came from using a number outside the situation it describes.
- **The real limit is often hidden in firmware.** The base joint was not limited by physics but by a speed cap I could not see until five curves hit the same ceiling.
- **Feedback only fixes what it can measure.** My heading loop worked well on heading and was blind to sideways slide.
- **A test is only as good as its signal-to-noise ratio.** My friction disturbance was about the size of the floor's own noise, so the tape-measure result could not tell the conditions apart.

## What's next

1. **Make the disturbance bigger than the noise.** Replace the paper with a much slicker plastic sheet so the drift reaches around 10 cm, well above the floor's noise.
2. **Close the position loop with a camera.** Use visual feedback, for example the robot's line-following or marker detection, to control sideways position as well as heading.
3. **Re-derive the base acceleration** from the raw angle data with a proper fit, and repeat each saturation run several times.
4. **Check the heat against torque.** Compare the 2.7 heating ratio with the squared ratio of my calculated shoulder torques.
5. **Repeat Stage 3 at more loads** to see whether overshoot grows linearly with load torque.

## Repository map

```
jetrover-control-experiments/
├── README.md                  ← this file
├── figures/                   ← every figure used above
├── code/
│   ├── esp32/                 ← PID on a simulated plant, PWM test
│   └── ros2_stage3_logger/    ← ROS 2 package: step_logger, temp_logger, heading_pid
├── data/                      ← raw CSV logs (see data/README.md)
└── media/                     ← photo and video links (see media/README.md)
```

**Platform:** Hiwonder JetRover (Mecanum base, six-axis bus-servo arm), ROS 2 · ESP32 Dev Module (Arduino IDE 2.3.10) · sigrok FX2 logic analyzer with PulseView.
