# README.MD

[![CI](https://github.com/frc4610/Rapid-React/actions/workflows/main.yml/badge.svg)](https://github.com/frc4610/Rapid-React/actions/workflows/main.yml)

## CAN Bus Ids

| Name | CAN Device | Id |
| --- | --- | --- |
| FRD | Talon FX | 1 |
| BRD | Talon FX | 2 |
| BLD | Talon FX | 3 |
| FLS | Talon FX | 4 |
| FRS | Talon FX | 5 |
| BRS | Talon FX | 6 |
| BLS | Talon FX | 7 |
| FLD | Talon FX | 8 |
| FRE | CANCoder | 9 |
| BRE | CANCoder | 10 |
| BLE | CANCoder | 11 |
| FLE | CANCoder | 12 |
| ARM | Talon FX | 13 |
| INTAKE | Talon FX | 14 |
| CANDLE | CANdle | 15 |
| PIGEON | Pigeon 2 | 16 |

## RoboRIO wiring
![RoboRIO_PINS](https://user-images.githubusercontent.com/98232538/162177238-916f35f6-f33d-463f-a577-e55d785b807d.png)

## Robot controls # Subject to change
1. LeftTrigger - Launches the balls from the intake when in firing position
2. RightTrigger - moves the arm beetween upper position and lower position
3. B-button - bypasses the ultrasonic allignment to fire balls
4. A-button - cuts power to drivetrain down to 50%
5. MenuButton - resets the field orientation 
6. D-Pad - rotates the bot in 90 degree increments
7. RightBumper - drives robot forward at climb speed
8. LeftBumper - drives backward straight

## Robot Checklist

- [ ] Arm articulation
- [ ] Intake motor
- [ ] Field orientation(Reset on `start` button)
- [ ] Limit power(Reset on `Bummper` button)
- [ ] Ultrasonic sensor
- [ ] RoboRIO(Power & rsl)
- [ ] PDP and breakers
- [ ] Wire connections
- [ ] Damage(METAL FLAKES!!!)

## Setting up module offsets

Now that we have code running on the robot, we can set up our module steering offsets. In order to do this we must have
our encoder values displayed to the dashboard.

> Before setting up module offsets ensure each offset is set to `-Math.toRadians(0.0)` and that code is deployed to the
> robot. This must be done each time offsets are determined.

1. Turn the robot on its side, so it is easy to move each module by hand.

> By default, module sensor information is displayed in the `Drivetrain` tab on ShuffleBoard.

2. Rotate each module so the bevel gear on the sides of each wheel are pointing to the robot's left.

> When aligning the wheels they must be as straight as possible. It is recommended to use a long straight edge such as
> a piece of 2x1 in order to make the wheels straight.

3. Record the angles of each module using the reading displayed on the dashboard.

4. Set the values of the `*_MODULE_STEER_OFFSET` constants in `Constants.Ids` to `-Math.toRadians(<the angle you recorded>)`
5. Re-deploy and try to drive the robot forwards. All wheels should stay parallel to each other.
6. Make sure all the wheels are spinning in the correct direction. If not, add 180 degrees to the offset of each wheel
that is spinning in the incorrect direction. (I.e. `-Math.toRadians(<angle> + 180.0))`)

## Tune PID for Rotation
- [ ] To tune start with a low P value (0.01).
- [ ] Multiply by 10 until the module starts oscilating around the set point
- [ ] Scale back by searching for the value (for example, if it starts oscillating at a P of 10, then try (10 -> 5 -> 7.5 -> etc)) until the module overshoots the setpoint but corrects with no oscillation.
- [ ] Repeat the process for D. The D value will basically help prevent the overshoot. Ignore I.

## Credits

- [ ] [Swerve Template by SDS](https://github.com/SwerveDriveSpecialties/swerve-template)
- [ ] [Modified SwerveLib by SDS](https://github.com/SwerveDriveSpecialties/swerve-lib)
- [ ] [Some util code from Team 2910](https://github.com/FRCTeam2910/Common)
 
