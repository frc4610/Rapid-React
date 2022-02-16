// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import swervelib.Mk3SwerveModuleHelper;
import swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Constants.*;

/*
// This can help when dealing with pathfinding
https://github.com/acmerobotics/road-runner-quickstart
*/

public class DrivetrainSubsystem extends SubsystemBase {
  // The formula for calculating the theoretical maximum velocity is:
  // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
  // pi
  // By default this value is setup for a Mk3 standard module using Falcon500s to
  // drive.
  // An example of this constant for a Mk4 L2 module with NEOs to drive is:
  // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
  // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));
  private static final int MAX_LATENCY_COMPENSATION_MAP_ENTRIES = 25;

  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  private final InterpolatingTreeMap<Pose2d> m_lagCompensationMap = InterpolatingTreeMap
      .createBuffer(MAX_LATENCY_COMPENSATION_MAP_ENTRIES);

  public static ShuffleboardTab m_DrivetrainTab;
  public static ShuffleboardTab m_DriveDataTab;
  private final NetworkTableEntry m_isFieldOriented;
  private static ShuffleboardLayout m_OdometryData;
  private static ShuffleboardLayout m_ChassisData;
  private static ShuffleboardLayout m_OtherData;
  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(0));

  public DrivetrainSubsystem() {
    m_DrivetrainTab = Shuffleboard.getTab("Drivetrain");
    m_DriveDataTab = Shuffleboard.getTab("Drive Data");
    m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
        // This parameter is optional, but will allow you to see the current state of
        // the module on the dashboard.
        m_DrivetrainTab.getLayout("Front Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0),
        // This can either be STANDARD or FAST depending on your gear configuration
        Mk3SwerveModuleHelper.GearRatio.STANDARD,
        // This is the ID of the drive motor
        FRONT_LEFT_MODULE_DRIVE_MOTOR,
        // This is the ID of the steer motor
        FRONT_LEFT_MODULE_STEER_MOTOR,
        // This is the ID of the steer encoder
        FRONT_LEFT_MODULE_STEER_ENCODER,
        // This is how much the steer encoder is offset from true zero (In our case,
        // zero is facing straight forward)
        FRONT_LEFT_MODULE_STEER_OFFSET);

    // We will do the same for the other modules
    m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
        m_DrivetrainTab.getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0),
        Mk3SwerveModuleHelper.GearRatio.STANDARD,
        FRONT_RIGHT_MODULE_DRIVE_MOTOR,
        FRONT_RIGHT_MODULE_STEER_MOTOR,
        FRONT_RIGHT_MODULE_STEER_ENCODER,
        FRONT_RIGHT_MODULE_STEER_OFFSET);

    m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
        m_DrivetrainTab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0),
        Mk3SwerveModuleHelper.GearRatio.STANDARD,
        BACK_LEFT_MODULE_DRIVE_MOTOR,
        BACK_LEFT_MODULE_STEER_MOTOR,
        BACK_LEFT_MODULE_STEER_ENCODER,
        BACK_LEFT_MODULE_STEER_OFFSET);

    m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
        m_DrivetrainTab.getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(6, 0),
        Mk3SwerveModuleHelper.GearRatio.STANDARD,
        BACK_RIGHT_MODULE_DRIVE_MOTOR,
        BACK_RIGHT_MODULE_STEER_MOTOR,
        BACK_RIGHT_MODULE_STEER_ENCODER,
        BACK_RIGHT_MODULE_STEER_OFFSET);

    zeroGyroscope();
    m_OdometryData = m_DriveDataTab.getLayout("Odometry Data", BuiltInLayouts.kList)
        .withSize(2, 2)
        .withPosition(0, 0);
    m_ChassisData = m_DriveDataTab.getLayout("Chassis Data", BuiltInLayouts.kList)
        .withSize(2, 2)
        .withPosition(2, 0);
    m_OtherData = m_DriveDataTab.getLayout("Other Data", BuiltInLayouts.kList)
        .withSize(2, 3)
        .withPosition(4, 0);
    m_OdometryData.addNumber("X", () -> {
      return getPose().getTranslation().getX();
    });
    m_OdometryData.addNumber("Y", () -> {
      return getPose().getTranslation().getY();
    });
    m_OdometryData.addNumber("Angle", () -> {
      return getPose().getRotation().getDegrees();
    });

    m_ChassisData.addNumber("X", () -> {
      return m_chassisSpeeds.vxMetersPerSecond;
    });
    m_ChassisData.addNumber("Y", () -> {
      return m_chassisSpeeds.vyMetersPerSecond;
    });
    m_ChassisData.addNumber("Z", () -> {
      return m_chassisSpeeds.omegaRadiansPerSecond;
    });

    m_isFieldOriented = m_OtherData.add("Field Oriented", true).getEntry();
    m_OtherData.addNumber("Gyro Rotation", () -> {
      return getGyroRotation().getDegrees();
    });
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the
   * robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    m_navx.zeroYaw();
  }

  public Rotation2d getGyroRotation() {
    if (m_navx.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    }
    // We have to invert the angle of the NavX so that rotating the robot
    // counter-clockwise makes the angle increase.
    return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public void drive(double translation_x, double translation_y, double rotation) {

    m_chassisSpeeds = m_isFieldOriented.getBoolean(true)
        ? ChassisSpeeds.fromFieldRelativeSpeeds(translation_x, translation_y, rotation, getGyroRotation())
        : new ChassisSpeeds(translation_x, translation_y, rotation);
  }

  public double getTurnRate() {
    return m_navx.getRate();
  }

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(pose, getGyroRotation());
  }

  public void updateOdometry(SwerveModuleState[] states) {
    m_odometry.update(getGyroRotation(), states); // Update Pose
    RobotContainer.dashboardField.setRobotPose(m_odometry.getPoseMeters()); // set field pose
    if (m_lagCompensationMap.size() > MAX_LATENCY_COMPENSATION_MAP_ENTRIES) {
      m_lagCompensationMap.getSample(m_lagCompensationMap.firstKey());
    }
    m_lagCompensationMap.addSample(Timer.getFPGATimestamp(), m_odometry.getPoseMeters()); // Add to interp map
  }

  public Pose2d getLagCompPose(double timestamp) {
    if (m_lagCompensationMap.isEmpty()) {
      return new Pose2d();
    }
    return m_lagCompensationMap.getSample(timestamp);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    m_chassisSpeeds = m_kinematics.toChassisSpeeds(states);
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);
    // FIXME: issues with clamping
    m_frontLeftModule.set(
        states[0].speedMetersPerSecond / Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND
            * Constants.Drivetrain.MAX_VOLTAGE,
        states[0].angle.getRadians());
    m_frontRightModule.set(
        states[1].speedMetersPerSecond / Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND
            * Constants.Drivetrain.MAX_VOLTAGE,
        states[1].angle.getRadians());
    m_backLeftModule.set(
        states[2].speedMetersPerSecond / Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND
            * Constants.Drivetrain.MAX_VOLTAGE,
        states[2].angle.getRadians());
    m_backRightModule.set(
        states[3].speedMetersPerSecond / Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND
            * Constants.Drivetrain.MAX_VOLTAGE,
        states[3].angle.getRadians());
    updateOdometry(states);
  }

  public void stopModules() {
    m_chassisSpeeds = new ChassisSpeeds(0, 0, 0);
  }
}
