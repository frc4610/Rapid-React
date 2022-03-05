# README.MD

## CAN Bus Ids

| Name | CAN Device | Id |
| --- | --- | --- |
| FRD1 | Talon FX | 1 |
| BRD2 | Talon FX | 2 |
| BLD3 | Talon FX | 3 |
| FLT4 | Talon FX | 4 |
| FRT5 | Talon FX | 5 |
| BRT6 | Talon FX | 6 |
| BLT7 | Talon FX | 7 |
| FLD8 | Talon FX | 8 |
| FRE | CANCoder | 9 |
| BRE | CANCoder | 10 |
| BLE | CANCoder | 11 |
| FLE | CANCoder | 12 |
| ARM | Talon FX | 13 |
| INTAKE | Talon FX | 14 |
| CANdle | CANdle | 15 |

## Firmware Version

naxX2-MXP (Gen2)->Firmware Version(4.0.442)
## Robot controls
1. LeftTrigger - Launches the balls from the intake when in firing position
2. RightTrigger - moves the arm beetween upper position and lower position
3. B-button - bypasses the ultrasonic allignment to fire balls
4. A-button - cuts power to drivetrain down to 50%
5. MenuButton - resets the field orientation 
6. D-Pad - rotates the bot in 90 degree increments
7. RightBumper - drives robot forward at climb speed
8. LeftBumper - drives backward straight



## Robot Checklist

## CANdle Status Lights

0. RoboRIO
1. Can Devices
2. Ultrasonic
3. Intake
4. Arm Position Verification

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
