// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import swervelib.Mk3SwerveModuleHelper;
import swervelib.SwerveModule;
import edu.wpi.first.math.controller.PIDController;
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

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0));
  private static final int MAX_LATENCY_COMPENSATION_MAP_ENTRIES = 25;

  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  private final InterpolatingTreeMap<Pose2d> m_lagCompensationMap = InterpolatingTreeMap
      .createBuffer(MAX_LATENCY_COMPENSATION_MAP_ENTRIES);

  public static ShuffleboardTab m_DrivetrainTab, m_DriveDataTab;
  private final NetworkTableEntry m_isFieldOriented;
  private static ShuffleboardLayout m_OdometryData, m_ChassisData, m_OtherData;
  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(0));

  private final double m_wheelMaxVoltage = 6.0;
  private double m_wheelVoltage = 6.0;
  private double m_lastWorldAccelX = -1.0, m_lastWorldAccelY = -1.0;
  private boolean m_didCollide = false;
  private double m_lastCollisionTime = 0.0;

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
        Ids.FRONT_LEFT.DRIVE_MOTOR,
        Ids.FRONT_LEFT.STEER_MOTOR,
        Ids.FRONT_LEFT.STEER_ENCODER,
        Ids.FRONT_LEFT.STEER_OFFSET);

    // We will do the same for the other modules
    m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
        m_DrivetrainTab.getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0),
        Mk3SwerveModuleHelper.GearRatio.STANDARD,
        Ids.FRONT_RIGHT.DRIVE_MOTOR,
        Ids.FRONT_RIGHT.STEER_MOTOR,
        Ids.FRONT_RIGHT.STEER_ENCODER,
        Ids.FRONT_RIGHT.STEER_OFFSET);

    m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
        m_DrivetrainTab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0),
        Mk3SwerveModuleHelper.GearRatio.STANDARD,
        Ids.BACK_LEFT.DRIVE_MOTOR,
        Ids.BACK_LEFT.STEER_MOTOR,
        Ids.BACK_LEFT.STEER_ENCODER,
        Ids.BACK_LEFT.STEER_OFFSET);

    m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
        m_DrivetrainTab.getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(6, 0),
        Mk3SwerveModuleHelper.GearRatio.STANDARD,
        Ids.BACK_RIGHT.DRIVE_MOTOR,
        Ids.BACK_RIGHT.STEER_MOTOR,
        Ids.BACK_RIGHT.STEER_ENCODER,
        Ids.BACK_RIGHT.STEER_OFFSET);

    zeroGyro();
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
  public void zeroGyro() {
    m_navx.reset();
  }

  public Rotation2d getGyroRotation() {
    // We have to invert the angle of the NavX so that rotating the robot
    // counter-clockwise makes the angle increase.

    if (ENABLE_MAGNETOMETER && m_navx.isMagnetometerCalibrated()) {
      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    }
    return Rotation2d.fromDegrees((INVERT_GYRO ? 360.0 : 0) - m_navx.getYaw());
  }

  public boolean getDidCollide() {
    double curWorldAccelX = m_navx.getWorldLinearAccelX();
    double curWorldAccelY = m_navx.getWorldLinearAccelY();

    double curJerkX = curWorldAccelX - m_lastWorldAccelX;
    double curJerkY = curWorldAccelY - m_lastWorldAccelY;

    m_lastWorldAccelX = curWorldAccelX;
    m_lastWorldAccelY = curWorldAccelY;

    if ((Math.abs(curJerkX) > COLLISION_THRESHOLD_DELTA) ||
        (Math.abs(curJerkY) > COLLISION_THRESHOLD_DELTA)) {
      m_didCollide = true;
      m_lastCollisionTime = Timer.getFPGATimestamp() + 5.0;
    } else if (Timer.getFPGATimestamp() > m_lastCollisionTime) {
      m_didCollide = false;
    }
    return m_didCollide;
  }

  public void drive(double translation_x, double translation_y, double rotation) {
    m_chassisSpeeds = m_isFieldOriented.getBoolean(true)
        ? ChassisSpeeds.fromFieldRelativeSpeeds(translation_x, translation_y, rotation, getGyroRotation())
        : new ChassisSpeeds(translation_x, translation_y, rotation);
  }

  public void drive(double translation_x, double translation_y, double rotation, boolean fieldOriented) {
    m_chassisSpeeds = fieldOriented
        ? ChassisSpeeds.fromFieldRelativeSpeeds(translation_x, translation_y, rotation, getGyroRotation())
        : new ChassisSpeeds(translation_x, translation_y, rotation);
  }

  public double getTurnRate() {
    return m_navx.getRate();
  }

  public double getDriveVoltage() {
    return m_wheelVoltage;
  }

  public void limitPower() {
    m_wheelVoltage = m_wheelMaxVoltage / 2;
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

  public void stopModules() {
    m_chassisSpeeds = new ChassisSpeeds(0, 0, 0);
  }

  // This is basic drift correction
  // I added it as it might be usefull later
  // Currently not used
  private static PIDController m_driftCorrectionPID = new PIDController(0.07, 0.00, 0.004);
  private static double m_desiredHeading;
  private static double m_prevXY = 0;

  public void getDriftCorrection(ChassisSpeeds speeds) {

    double curXY = Math.abs(speeds.vxMetersPerSecond) + Math.abs(speeds.vyMetersPerSecond);

    if (Math.abs(speeds.omegaRadiansPerSecond) > 0.0 || m_prevXY <= 0)
      m_desiredHeading = getPose().getRotation().getDegrees();

    else if (curXY > 0)
      speeds.omegaRadiansPerSecond += m_driftCorrectionPID.calculate(getPose().getRotation().getDegrees(),
          m_desiredHeading);

    m_prevXY = curXY;
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Motor.MAX_VELOCITY_METERS_PER_SECOND);
    // FIXME: issues with clamping
    m_frontLeftModule.set(
        states[0].speedMetersPerSecond / Motor.MAX_VELOCITY_METERS_PER_SECOND
            * m_wheelVoltage,
        states[0].angle.getRadians());
    m_frontRightModule.set(
        states[1].speedMetersPerSecond / Motor.MAX_VELOCITY_METERS_PER_SECOND
            * m_wheelVoltage,
        states[1].angle.getRadians());
    m_backLeftModule.set(
        states[2].speedMetersPerSecond / Motor.MAX_VELOCITY_METERS_PER_SECOND
            * m_wheelVoltage,
        states[2].angle.getRadians());
    m_backRightModule.set(
        states[3].speedMetersPerSecond / Motor.MAX_VELOCITY_METERS_PER_SECOND
            * m_wheelVoltage,
        states[3].angle.getRadians());
    updateOdometry(states);

    RobotContainer.dashboardField.setRobotPose(m_odometry.getPoseMeters()); // set field pose
    m_wheelVoltage = m_wheelMaxVoltage;
  }

  @Override
  public void simulationPeriodic() {
    periodic();
  }

  public SwerveModule[] getSwerveModules() {
    return new SwerveModule[] { m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule };
  }
}
