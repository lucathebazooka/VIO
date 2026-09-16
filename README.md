# VIO

One-sentence description + a photo or GIF of it working (GIFs are gold).

## What it does
This drone navigates without GPS using Visual-Inertial Odometry (VIO): a downward-facing stereo camera and an IMU are fused onboard a Raspberry Pi 5 to estimate the drone's position and motion in real time. This enables autonomous flight in GPS-denied or GPS-degraded environments such as indoors, warehouses, disaster zones, defence settings, as well as in tasks like infrastructure inspection where navigating relative to the immediate surroundings matters more than global coordinates.

## Hardware
- Sing Board Computer: Raspberry Pi 5 8GB
- AI Accelerator: Raspberry Pi AI Hat+ 13 TOPS
- Flight ControllerL DAKEFPV H743 30x30
- Propellers: Gemfan 8x4x3
- Arms (Carbon Fibre Tube): 3K CF tube OD: 10mm ID: 6mm
- Motors: Anoel 2812 1115KV
- Down Firing Stereo Camera: 1MP OV9281 Global Shutter Binocular Synchronous
- ESC: DAKEFPV 8S 70A BLHeli_S 30x30
- External IMU: BMI088a
- Frame: 3D printed from PLA with design files to be uploaded soon

- Wiring diagram or photo

## How it works
The interesting bit. Architecture in plain English:
sensors → processing → decisions → actuators.
A simple diagram helps a lot.
![flow diagrom of how the VIO system works](docs/system_diagram/VIO_diagram.png)

## Design decisions
For each major choice: what I picked, what the alternative was, why.
e.g. "Complementary filter over Kalman: simpler to tune, and at 100Hz
the accuracy difference didn't matter for this task."

(All design decisions for building of the drone)

## What went wrong
The most valuable section. Honest failures and how you diagnosed them.
e.g. "Robot oscillated wildly — traced to IMU mounted too far from
the wheel axle, amplifying angular noise."



## Results
Numbers, tables, videos. For the evals-style project this is the
core: conditions tested × success rates.

## What I'd do differently / next steps
Shows you're still thinking. 2–3 bullets.
