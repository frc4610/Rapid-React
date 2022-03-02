// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.SwerveConfig;

public final class Constants {
  public static final String VERSION = "Version 0.1.4";

  public final static class Ids {
    // Analog
    public static final int LEFT_ULTRASONIC = 1;
    public static final int RIGHT_ULTRASONIC = 0;

    // CAN bus
    public static final int LED_CANDLE = 9;
    public static final int ARM = 13;
    public static final int INTAKE = 14;

    public static final SwerveConfig FRONT_LEFT = new SwerveConfig(8, 4, 11, -Math.toRadians(149.85));
    public static final SwerveConfig FRONT_RIGHT = new SwerveConfig(1, 5, 9, -Math.toRadians(181.40));
    public static final SwerveConfig BACK_LEFT = new SwerveConfig(3, 7, 12, -Math.toRadians(128.76));
    public static final SwerveConfig BACK_RIGHT = new SwerveConfig(2, 6, 10, -Math.toRadians(30.05));
  }

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

  public final static class Arm {
    public static final double TRAVEL_UP_POWER = 0.4;
    public static final double TRAVEL_DOWN_POWER = 0.2;
    public static final double TRAVEL_DIFFRENCE = 0.15;
    public static final double UP_POSITION = 38000;
    public static final double DOWN_POSITION = 2000;

  }

  public final static class Intake {
    public static final double POWER = 0.69;
  }

  public static final boolean ENABLE_MAGNETOMETER = false;
  public static final boolean INVERT_GYRO = true;
  // FIXME: Calculate the threshold manually
  public static final double COLLISION_THRESHOLD_DELTA = 0.5;

  // The left-to-right distance between the drivetrain wheels
  public static final double TRACKWIDTH_METERS = 0.65532; // 0.56 //21.5"
  // The front-to-back distance between the drivetrain wheels.
  public static final double WHEELBASE_METERS = 0.76581; // 0.545 // 25 2/16"

  public final static class Field {
    public static final double FIELD_LENGTH = Units.inchesToMeters(54.0 * 12.0);
    public static final double FIELD_WIDTH = Units.inchesToMeters(27.0 * 12.0);
    public static final double HANGAR_LENGTH = Units.inchesToMeters(128.75);
    public static final double HANGAR_WIDTH = Units.inchesToMeters(116.0);
  }
}
