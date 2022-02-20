// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public final static class Limelight {
    public static final double TARGET_HEIGHT = 36.0;
    public static final double LIMELIGHT_HEIGHT = 9;
    public static final double LIMELIGHT_ANGLE = Math.toRadians(45.0);
    public static final double TARGET_ALLOWABLE_ERROR = Math.toRadians(2.0);
  }

  public final static class Ultrasonic {
    public static final double WIDTH_INCH = 26.95;
  }

  public final static class Controller {
    public static final double XBOX_DEADBAND = 0.05;
  }

  // Motor Specific
  public final static class Motor {
    public static final double MAX_VOLTAGE = 6.0;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
        SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
        SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

    // SDS Billet Wheels 4"D X 1"W
    // 8.16:1 Gear Ratio
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    public static final double WHEEL_WIDTH = Units.inchesToMeters(1.0);

    public static final double DRIVE_GEAR_RATIO = 8.16;
    public static final double ANGLE_GEAR_RATIO = 12.8;
  }

  public final static class Autonomous {
    public static final double PX_CONTROLLER = 4;
    public static final double PY_CONTROLLER = 4;
    public static final double PTHETA_CONTROLLER = 2;

    // theoretical values do not use mk3 as they are the motor
    // FIXME: calculate the max
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 3;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND = 3;

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = Math.PI;
    public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND = Math.PI;
    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints PTHETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
        MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND);

  }

  public static final boolean INVERT_GYRO = true;
  public static final double COLLISION_TRESHOLD_DELTA = 0.5f;

  // The left-to-right distance between the drivetrain wheels
  public static final double TRACKWIDTH_METERS = 0.555; // 0.56
  // The front-to-back distance between the drivetrain wheels.
  public static final double WHEELBASE_METERS = 0.55; // 0.545

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 8; // Set front left module drive motor ID - 4
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 4; // Set front left module steer motor ID - 8
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 11; // Set front left steer encoder ID - 12
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(149.85);

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1; // Set front right drive motor ID - 1
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5; // Set front right steer motor ID - 5
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 9; // Set front right steer encoder ID - 9
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(181.40);

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3; // Set back left drive motor ID - 3
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 7; // Set back left steer motor ID - 7
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 12; // Set back left steer encoder ID - 11
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(128.76);

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 2; // Set back right drive motor ID - 2
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 6; // Set back right steer motor ID - 6
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 10; // Set back right steer encoder ID - 10
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(30.05);
}
