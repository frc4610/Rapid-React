package frc.robot;

import swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.utils.SwerveConfig;

public final class Constants {

  private static final ShuffleboardTab m_tab = Shuffleboard.getTab("Constants");

  public static boolean USE_ONE_CONTROLLER = false;
  public static final String VERSION = "Version 0.2.0";
  public static final String RIO_IP = "http://172.22.11.2:1250";

  public final static class Ids {
    // PWM
    public static final int PWM_LED_STRIP = 0;

    // Analog
    public static final int LEFT_ULTRASONIC = 0;
    public static final int RIGHT_ULTRASONIC = 1;

    // CAN bus
    public static final int ARM = 13;
    public static final int INTAKE = 14;
    public static final int LED_CANDLE = 15;

    public static final SwerveConfig FRONT_LEFT = new SwerveConfig(8, 4, 11, -Math.toRadians(149.85));
    public static final SwerveConfig FRONT_RIGHT = new SwerveConfig(1, 5, 9, -Math.toRadians(181.40));
    public static final SwerveConfig BACK_LEFT = new SwerveConfig(3, 7, 12, -Math.toRadians(128.76));
    public static final SwerveConfig BACK_RIGHT = new SwerveConfig(2, 6, 10, -Math.toRadians(30.05));
  }

  public final static class Ultrasonic {
    public static final double LENGTH_FROM_SIDE = 6.0; // inches from front side
    public static final double WIDTH_INCH = 25.15;
    public static final double ANGULAR_THRESHOLD = 5.0;
    public static final double MIN_DISTANCE = 12.0;
    public static final double MAX_DISTANCE = 20.0;
  }

  public final static class Controller {
    public static final double XBOX_DEADBAND = 0.2;
  }

  // Motor Specific
  public final static class Motor {
    public static final int MAX_RPM = 6380;
    public static final double MAX_VELOCITY_MPS = MAX_RPM / 60.0 *
        SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
        SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;

    public static final double MAX_ANGULAR_VELOCITY_RPS = MAX_VELOCITY_MPS /
        Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

    public static final double TURN_TOLERANCE = 3; // degrees

    // SDS Billet Wheels 4"D X 1"W
    // 8.16:1 Gear Ratio
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    public static final double WHEEL_WIDTH = Units.inchesToMeters(1.0);

    public static final double DRIVE_GEAR_RATIO = 8.16;
    public static final double ANGLE_GEAR_RATIO = 12.8;
    public static final double DEFAULT_MAX_POWER = 6.0;
    public static final NetworkTableEntry MAX_POWER = m_tab.add("Motor max power", DEFAULT_MAX_POWER).getEntry();
  }

  public final static class Auto {
    public static final double DRIVE_POWER = 4;

    // FIXME: Use Azmith to calculate th right meters
    public static final double PXY_CONTROLLER = 0.5;
    public static final double IXY_CONTROLLER = 0.1;
    public static final double DXY_CONTROLLER = 0.0;

    public static final double PTHETA_CONTROLLER = 2.5;
    public static final double ITHETA_CONTROLLER = 0.0;
    public static final double DTHETA_CONTROLLER = 0.0;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
        Motor.MAX_ANGULAR_VELOCITY_RPS, Motor.MAX_ANGULAR_VELOCITY_RPS); //  accel * 0.9

  }

  public final static class Arm {
    public static final double DEFAULT_TRAVEL_UP_POWER = 0.4;
    public static final NetworkTableEntry TRAVEL_UP_POWER = m_tab.add("Arm up power", DEFAULT_TRAVEL_UP_POWER)
        .getEntry(); // .35
    public static final double DEFAULT_TRAVEL_DOWN_POWER = 0.35;
    public static final NetworkTableEntry TRAVEL_DOWN_POWER = m_tab.add("Arm down power", DEFAULT_TRAVEL_DOWN_POWER)
        .getEntry();
    public static final double DEFAULT_TRAVEL_DISTANCE = 0.2;
    public static final NetworkTableEntry TRAVEL_DIFFRENCE = m_tab.add("Arm travel diffrence", DEFAULT_TRAVEL_DISTANCE)
        .getEntry();
    public static final double ABS_UP_POSITION = 60000; // Range from RNG - MAX
    public static final double UP_POSITION = ABS_UP_POSITION - 1500; // Range from RNG - MAX
    public static final double DOWN_POSITION = 100; // Enough to hold the bot down
  }

  public final static class Intake {
    public static final NetworkTableEntry POWER_OUT = m_tab.add("Intake power out", 0.8).getEntry();
    public static final NetworkTableEntry POWER_IN = m_tab.add("Intake power in", 0.55).getEntry();
  }

  public static final boolean ENABLE_MAGNETOMETER = false;
  public static final boolean INVERT_GYRO = true;

  // Not used currently
  public static final double COLLISION_THRESHOLD_DELTA = 0.5;

  // Robot is 31" long and 28" wide
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
