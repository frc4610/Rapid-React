// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.56; // 0.56
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.545; // 0.545

    //public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXME Set Pigeon ID -  Removed for

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 4; // Set front left module drive motor ID - 4
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 8; // Set front left module steer motor ID - 8
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 12; // Set front left steer encoder ID - 12
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1; // Set front right drive motor ID - 1
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5; // Set front right steer motor ID - 5
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 9; // Set front right steer encoder ID - 9 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3; // Set back left drive motor ID - 3
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 7; // Set back left steer motor ID - 7
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11; // Set back left steer encoder ID - 11
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 2; // Set back right drive motor ID - 2
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 6; // Set back right steer motor ID - 6
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 10; // Set back right steer encoder ID - 10
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set back right steer offset
}
