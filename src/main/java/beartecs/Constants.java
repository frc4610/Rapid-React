package beartecs;

import beartecs.CAN.CANConfig;
import beartecs.configs.PidConfig;
import beartecs.configs.ProfiledPidConfig;
import beartecs.configs.SwerveConfig;
import beartecs.swerve.config.Mk3ModuleConfiguration;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public final class Constants {

  private static final ShuffleboardTab m_tab = Shuffleboard.getTab("Constants");
  public static final String RIO_IP = "http://172.22.11.2:1250";
  public static final double CHECK_VOLTAGE = 12.2;

  public final static class Ids {
    // DIO
    public static final int DIO_TOP_LIMITSWTICH = 3;
    public static final int DIO_BOTTOM_LIMITSWTICH = 2;

    // PWM
    public static final int PWM_LED_STRIP = 0;

    // Analog
    public static final int LEFT_ULTRASONIC = 0;
    public static final int RIGHT_ULTRASONIC = 1;

    // CAN bus
    public static final String CANIVORE_NAME = "CANivore_Swerve";
    public static final CANConfig ARM = new CANConfig(13, CANIVORE_NAME);
    public static final CANConfig INTAKE = new CANConfig(14, CANIVORE_NAME);
    public static final CANConfig LED_CANDLE = new CANConfig(15, CANIVORE_NAME);
    public static final CANConfig PIGEON = new CANConfig(16);

    // TODO: Reconfigure abs encoder offsets
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
    public static final double XBOX_DEADBAND = 0.08; // Fixes controller diffrences
  }

  public final static class Camera {
    public static final int IMG_WIDTH = 1280;
    public static final int IMG_HEIGHT = 720;
  }

  // Motor Specific
  public final static class Motor {
    public static final boolean OPEN_LOOP = false;
    public static final boolean DEFENSIVE = true;
    public static final int MAX_RPM = 6380;
    public static final double MAX_VELOCITY_MPS = MAX_RPM / 60.0 *
        Mk3ModuleConfiguration.STANDARD.getDriveReduction() *
        Mk3ModuleConfiguration.STANDARD.getWheelDiameter() * Math.PI;

    public static final double MAX_ANGULAR_VELOCITY_RPS = MAX_VELOCITY_MPS /
        Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

    public static final double TURN_TOLERANCE = 12.0; // degrees
    public static final double TALON_TPR = 2048.0; // Ticks per Rotation

    // SDS Billet Wheels 4"D X 1"W
    // 8.16:1 Gear Ratio
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    public static final double WHEEL_WIDTH = Units.inchesToMeters(1.0);

    public static final double DRIVE_GEAR_RATIO = 8.16;
    public static final double ANGLE_GEAR_RATIO = 12.8;
    public static final double MAX_POWER = 12;
    public static final double DRIVE_POWER = 6;
  }

  public final static class Auto {
    public static final double DRIVE_POWER = 6.5;
    // Constraint for the motion profilied robot angle controller
    private static final double THETA_CONSTRAINT_SCALAR = 0.8;
    public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
        Motor.MAX_ANGULAR_VELOCITY_RPS,
        Motor.MAX_ANGULAR_VELOCITY_RPS * THETA_CONSTRAINT_SCALAR);

    public static final PidConfig PID_XY = new PidConfig(3.0, 0.0, 0.02);

    // kp: 0.05
    // Motor Feedback is using onboard sensor which is velocity corrected per 100ms(normal full rotation is 2048) yet the output's max is 1023
    // divide by 12 to convert from volts to percent output for CTRE // If we were using .set for arb ff
    public static final double STATIC_GAIN = 0.49;
    public static final double VELOCITY_GAIN = 2.1;
    public static final double ACCELERATION_GAIN = 0.27;

    public static final ProfiledPidConfig PID_THETA = new ProfiledPidConfig(1.269, 0.0, 0.02, THETA_CONSTRAINTS);
  }

  public final static class Arm {
    public static final double DEFAULT_TRAVEL_UP_POWER = 0.369;
    public static final NetworkTableEntry TRAVEL_UP_POWER = m_tab.add("Arm up power", DEFAULT_TRAVEL_UP_POWER)
        .getEntry(); // .35
    public static final double DEFAULT_TRAVEL_DOWN_POWER = 0.35;
    public static final NetworkTableEntry TRAVEL_DOWN_POWER = m_tab.add("Arm down power", DEFAULT_TRAVEL_DOWN_POWER)
        .getEntry();
    public static final double DEFAULT_TRAVEL_DISTANCE = 0.25;
    public static final NetworkTableEntry TRAVEL_DIFFERENCE = m_tab
        .add("Arm travel difference", DEFAULT_TRAVEL_DISTANCE)
        .getEntry();
    /*
    UP: 32340
    POINT:32018 //134.167236328125*
    DOWN:0 //12.697265624999998*
    */
    public static final double ABS_UP_POSITION = 35000; // Range from RNG - MAX
    public static final double UP_POSITION = ABS_UP_POSITION - 1500; // Range from RNG - MAX
    public static final double DOWN_POSITION = 100; // Enough to hold the bot down

    // Right angle might be 1:14
    // 9 to 84 connected to 30 which leads to 76
    public static final double GEAR_RATIO = (22.0 / 64.0) * (1.0 / 15.0); // Custom gearbox -> 9:84->30:76
    // Might need more tuning but works
    public static final ProfiledPidConfig ARM_PID = new ProfiledPidConfig(0.55, 0.0, 0.5,
        new TrapezoidProfile.Constraints(3.5, 1.5));
  }

  public final static class Intake {
    public static final double AUTO_POWER_OUT = 0.6;
    public static final double AUTO_POWER_IN = 0.35;

    public static final double DEFAULT_POWER_OUT = 0.5;
    public static final NetworkTableEntry POWER_OUT = m_tab.add("Intake power out", DEFAULT_POWER_OUT).getEntry();
    public static final double DEFAULT_POWER_IN = 0.5;
    public static final NetworkTableEntry POWER_IN = m_tab.add("Intake power in", DEFAULT_POWER_IN).getEntry();
  }

  // Not used currently
  public static final double COLLISION_THRESHOLD_DELTA = 0.5;

  // Robot is 31" long and 28" wide
  // The left-to-right distance between the drivetrain wheels
  public static final double TRACKWIDTH_METERS = 0.65532; // 0.56 //21.5"
  // The front-to-back distance between the drivetrain wheels.
  public static final double WHEELBASE_METERS = 0.76581; // 0.545 // 25.125"

  // Can be used to calculate special limits 
  public static final double MASS = Units.lbsToKilograms(140);
  public static final double INTERTIA = 1.0 / 12.0 * MASS * Math.pow((TRACKWIDTH_METERS * 1.1), 2) * 2;

  public final static class Field {
    public static final double FIELD_LENGTH = Units.inchesToMeters(54.0 * 12.0);
    public static final double FIELD_WIDTH = Units.inchesToMeters(27.0 * 12.0);
    public static final double HANGAR_LENGTH = Units.inchesToMeters(128.75);
    public static final double HANGAR_WIDTH = Units.inchesToMeters(116.0);
    public static final Translation2d HUB_CENTER_TRANSLATION = new Translation2d(8.2296, 4.1148);
    public static final Pose2d HUB_CENTER_POSE = new Pose2d(HUB_CENTER_TRANSLATION, new Rotation2d(0));
  }

  // Will fix some issues with abs positions when abs pos fails
  public final static boolean ENABLE_ABS_ENCODER_POS_ERROR_CHECKS = true;
  public final static int ABS_ENCODER_ERROR_RETRY_COUNT = 3;

  public final static boolean ALIGN_RANGE_ENABLE = true;
  public final static boolean BOOT_TO_ABS = true;
  public final static boolean ENABLE_ABS_SET_MOTOR = true;

  public final class Sim {
    static public final int STATUS_FRAME_PERIOD_MS = 20; // Falcons update every 250 ms but in sim we want it to update faster
    static public final double SAMPLE_RATE_SEC = 0.02;
  }

  public final static boolean ENABLE_DS_LOG_SAVE = false;
  public final static int LOG_EXPIRATION_IN_HRS = 24 * 2;
}
