// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import swervelib.Gyroscope;
import swervelib.GyroscopeHelper;
import swervelib.SwerveModule;
import swervelib.config.Mk3SwerveModuleHelper;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotContainer;
import frc.robot.utils.*;
import frc.robot.utils.math.InterpolatingTreeMap;
import frc.robot.utils.math.MathUtils;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Constants.*;
/*
// This can help when dealing with pathfinding
https://github.com/acmerobotics/road-runner-quickstart
*/

public class DrivetrainSubsystem extends BaseSubsystem {

  private static final Translation2d m_frontLeftModulePosition = new Translation2d(TRACKWIDTH_METERS / 2,
      WHEELBASE_METERS / 2);
  private static final Translation2d m_frontRightModulePosition = new Translation2d(TRACKWIDTH_METERS / 2,
      -WHEELBASE_METERS / 2);
  private static final Translation2d m_backLeftModulePosition = new Translation2d(-TRACKWIDTH_METERS / 2,
      WHEELBASE_METERS / 2);
  private static final Translation2d m_backRightModulePosition = new Translation2d(-TRACKWIDTH_METERS / 2,
      -WHEELBASE_METERS / 2);

  public static final Translation2d[] m_modulePositions = {
      m_frontLeftModulePosition,
      m_frontRightModulePosition,
      m_backLeftModulePosition,
      m_backRightModulePosition
  };
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // Front left
      m_frontLeftModulePosition,
      // Front right
      m_frontRightModulePosition,
      // Back left
      m_backLeftModulePosition,
      // Back right
      m_backRightModulePosition);
  private static final int MAX_LATENCY_COMPENSATION_MAP_ENTRIES = 25;

  private final Gyroscope m_gyro;

  private final InterpolatingTreeMap<Pose2d> m_lagCompensationMap = InterpolatingTreeMap
      .createBuffer(MAX_LATENCY_COMPENSATION_MAP_ENTRIES);

  public static ShuffleboardTab m_DrivetrainTab, m_DriveDataTab;
  private final NetworkTableEntry m_isFieldOriented;
  private static ShuffleboardLayout m_OdometryData, m_ChassisData, m_OtherData;
  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule;

  private SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(Auto.STATIC_GAIN, Auto.VELOCITY_GAIN,
      Auto.ACCELERATION_GAIN);

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private final SwerveDriveOdometry m_odometry;

  private double m_speedModifier = 1.0;

  public DrivetrainSubsystem() {
    m_gyro = GyroscopeHelper.createNavXMXP(); // 8.5cm from front bar // 30cm in the center of the bar
    m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroRotation());
    resetPose(new Pose2d(7, 2, Rotation2d.fromDegrees(-90)));

    m_DrivetrainTab = addTab("Drivetrain");
    m_DriveDataTab = addTab("Drive Data");
    m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
        // This parameter is optional, but will allow you to see the current state of
        // the module on the dashboard.
        m_DrivetrainTab.getLayout("Front Left Module", BuiltInLayouts.kList)
            .withSize(2, 2)
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
            .withSize(2, 2)
            .withPosition(2, 0),
        Mk3SwerveModuleHelper.GearRatio.STANDARD,
        Ids.FRONT_RIGHT.DRIVE_MOTOR,
        Ids.FRONT_RIGHT.STEER_MOTOR,
        Ids.FRONT_RIGHT.STEER_ENCODER,
        Ids.FRONT_RIGHT.STEER_OFFSET);

    m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
        m_DrivetrainTab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withPosition(4, 0),
        Mk3SwerveModuleHelper.GearRatio.STANDARD,
        Ids.BACK_LEFT.DRIVE_MOTOR,
        Ids.BACK_LEFT.STEER_MOTOR,
        Ids.BACK_LEFT.STEER_ENCODER,
        Ids.BACK_LEFT.STEER_OFFSET);

    m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
        m_DrivetrainTab.getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 2)
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
    m_gyro.zeroGyroscope();
    m_odometry.resetPosition(
        new Pose2d(m_odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)),
        getGyroRotation());
  }

  public Rotation2d getGyroRotation() {
    return m_gyro.getGyroRotation();
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

  // Calcs from gains to drive accuratly
  // Ex: Drive 1 meter with drive fails where this would not
  private double getVelocityToVoltage(double speedMetersPerSecond) {
    double wheel_voltage = Motor.MAX_POWER.getDouble(Motor.DEFAULT_MAX_POWER);
    if (Motor.ENABLE_FF)
      return speedMetersPerSecond / Motor.MAX_VELOCITY_MPS * wheel_voltage;
    return MathUtils.clamp(m_feedForward.calculate(speedMetersPerSecond), -wheel_voltage, wheel_voltage);
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

  public void setSpeedModifier(double mod) {
    m_speedModifier = mod;
  }

  public void setModuleStates(SwerveModuleState[] states) {
    m_chassisSpeeds = m_kinematics.toChassisSpeeds(states);
  }

  public void setModuleStates(ChassisSpeeds state) {
    m_chassisSpeeds = state;
  }

  public void stopModules() {
    m_chassisSpeeds = new ChassisSpeeds(0, 0, 0);
  }

  @Override
  public void periodic() {
    updateOdometry(getSwerveModuleStates()); // Update odometry based off wheel states, NOT requested chassis speeds
    if (Math.abs(m_chassisSpeeds.vxMetersPerSecond) == 0 && Math.abs(m_chassisSpeeds.vyMetersPerSecond) == 0
        && Math.abs(m_chassisSpeeds.omegaRadiansPerSecond) == 0 && Motor.DEFENSIVE) {
      m_frontLeftModule.set(
          0.0, Math.toRadians(-45));
      m_frontRightModule.set(
          0.0, Math.toRadians(45));
      m_backLeftModule.set(
          0.0, Math.toRadians(45));
      m_backRightModule.set(
          0.0, Math.toRadians(-45));
    } else {

      // Speed Motifier
      m_chassisSpeeds.vxMetersPerSecond *= m_speedModifier;
      m_chassisSpeeds.vyMetersPerSecond *= m_speedModifier;
      m_chassisSpeeds.omegaRadiansPerSecond *= m_speedModifier;

      SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(states, Motor.MAX_VELOCITY_MPS);

      m_frontLeftModule.set(getVelocityToVoltage(
          states[0].speedMetersPerSecond),
          states[0].angle.getRadians());
      m_frontRightModule.set(getVelocityToVoltage(
          states[1].speedMetersPerSecond),
          states[1].angle.getRadians());
      m_backLeftModule.set(getVelocityToVoltage(
          states[2].speedMetersPerSecond),
          states[2].angle.getRadians());
      m_backRightModule.set(getVelocityToVoltage(
          states[3].speedMetersPerSecond),
          states[3].angle.getRadians());
      //updateOdometry(states);
    }
    Pose2d[] modulePose = { null, null, null, null };
    var swerveModules = getSwerveModuleStates();
    // Update the poses for the swerveModules. Note that the order of rotating the position and then
    // adding the translation matters
    for (int i = 0; i < swerveModules.length; i++) {
      var modulePositionFromChassis = m_modulePositions[i]
          .rotateBy(getGyroRotation())
          .plus(getPose().getTranslation());

      // Module's heading is it's angle relative to the chassis heading
      modulePose[i] = new Pose2d(modulePositionFromChassis,
          swerveModules[i].angle.plus(getPose().getRotation()));
    }
    RobotContainer.telemetry.setSwerveModulePoses(modulePose);
    RobotContainer.telemetry.setActualPose(m_odometry.getPoseMeters());
    RobotContainer.telemetry.update();
  }

  public SwerveModule[] getSwerveModules() {
    return new SwerveModule[] { m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule };
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeftModule.getState(),
        m_frontRightModule.getState(),
        m_backLeftModule.getState(),
        m_backRightModule.getState()
    };
  }
}
