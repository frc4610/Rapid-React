// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public final static class Limelight {
    public static final double TARGET_HEIGHT = 36.0;
    public static final double LIMELIGHT_HEIGHT = 9;
    public static final double LIMELIGHT_ANGLE = Math.toRadians(45.0);
  }

  public final static class Controller {
    public static final double X_AXIS_DEADBAND = 0.05;
    public static final double Y_AXIS_DEADBAND = 0.05;
    public static final double Z_AXIS_DEADBAND = 0.05;
  }

  // The left-to-right distance between the drivetrain wheels
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.555; // 0.56
  // The front-to-back distance between the drivetrain wheels.
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.55; // 0.545

  public static final double GYRO_ADJUST_COEFFICENT = 0.01;

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
