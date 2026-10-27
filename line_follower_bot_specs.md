# Line-Follower Robot — Hardware Specifications

## 1. Overview

| Item | Specification |
|---|---|
| Main controller | Arduino Nano |
| Line sensor | Robo Junkies 7-array analog sensor |
| Motor driver | TB6612FNG dual H-bridge |
| Wireless module | None |
| Motors | 2 × N20 DC gear motors |
| Motor rating | 6 V, 300 RPM |
| Total bot length | 21 cm |

## 2. Robo Junkies 7-Array Analog Sensor

Connect the sensor outputs to the Arduino Nano as follows:

| Sensor pin | Arduino Nano pin |
|---|---|
| S1 | A0 |
| S2 | A1 |
| S3 | A2 |
| S4 | A3 |
| S5 | A4 |
| S6 | A5 |
| S7 | A6 |
| VCC | 5V, only if supported by the sensor board |
| GND | GND |

**Important:** A6 on the classic Arduino Nano is analog-input-only; it cannot be used as a digital I/O pin. Confirm the sensor board's operating-voltage requirements before connecting VCC.

## 3. TB6612FNG Motor Driver

### Arduino-to-driver signal wiring

| TB6612FNG pin | Arduino Nano pin |
|---|---|
| PWMA | D5 |
| AIN1 | D3 |
| AIN2 | D4 |
| PWMB | D6 |
| BIN1 | D7 |
| BIN2 | D9 |
| STBY | D8 |
| VCC | 5V |
| GND | Common GND |
| VM | Regulated 6V motor supply |

### Motor connections

| Driver output | Motor |
|---|---|
| AO1, AO2 | Right N20 motor |
| BO1, BO2 | Left N20 motor |

The left/right motor direction depends on the motor lead polarity and the driver's input logic. If a motor runs opposite to the expected direction, swap that motor's two output leads or invert its direction in software.

## 4. Power and Ground

- Use a regulated 6V supply for the TB6612FNG **VM** motor-supply input.
- Connect the Arduino, motor driver, and sensor grounds together.
- Connect TB6612FNG **VCC** to the logic supply (5V as specified).
- Do not power the motors from the Arduino Nano's 5V pin.
- Ensure the motor supply can handle the motors' startup/stall current. Check the TB6612FNG board's ratings and wiring before powering the robot.

## 5. Mechanical Specification

- **Total bot length:** 21 cm
- **Motor type:** N20 DC gear motors
- **Motor voltage:** 6V
- **Rated speed:** 300 RPM

Width, wheel diameter, wheelbase, chassis material, and sensor-to-ground clearance have not been specified.

## 6. Pin Summary

| Arduino Nano pin | Connected component |
|---|---|
| A0 | Sensor S1 |
| A1 | Sensor S2 |
| A2 | Sensor S3 |
| A3 | Sensor S4 |
| A4 | Sensor S5 |
| A5 | Sensor S6 |
| A6 | Sensor S7 |
| D3 | TB6612FNG AIN1 (Right Motor In 1) |
| D4 | TB6612FNG AIN2 (Right Motor In 2) |
| D5 | TB6612FNG PWMA (Right Motor PWM) |
| D6 | TB6612FNG PWMB (Left Motor PWM) |
| D7 | TB6612FNG BIN1 (Left Motor In 1) |
| D8 | TB6612FNG STBY |
| D9 | TB6612FNG BIN2 (Left Motor In 2) |
| D13 | Onboard LED (Junction Indicator) |
| 5V | Sensor VCC (if supported), TB6612FNG VCC |
| GND | Common ground |

## 7. Pre-Power Checklist

- [ ] Verify the sensor board's VCC rating.
- [ ] Confirm all grounds are connected.
- [ ] Verify motor polarity and left/right output mapping.
- [ ] Ensure the 6V motor supply is regulated and has sufficient current capacity.
- [ ] Keep the robot's wheels lifted for the first motor-direction test.

