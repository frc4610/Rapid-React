// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.*;
import edu.wpi.first.wpilibj.SPI;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 6.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
          SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;

  public static final double MAX_VELOCITY_METERS_PER_SECOND_SQRT = Math.sqrt(MAX_VELOCITY_METERS_PER_SECOND);
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  private Rotation2d m_targetHeading;

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
  
  private final SwerveDriveOdometry m_odometer = new SwerveDriveOdometry(m_kinematics, new Rotation2d(0));
  
  public DrivetrainSubsystem() {
    m_DrivetrainTab = Shuffleboard.getTab("Drivetrain");
    m_DriveDataTab = Shuffleboard.getTab("Drive Data");
    m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
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
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
            m_DrivetrainTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
            m_DrivetrainTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
            m_DrivetrainTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );

    zeroGyroscope();
    m_targetHeading = getGyroRotation();
    m_OdometryData = m_DriveDataTab.getLayout("Odometry Data", BuiltInLayouts.kList)
      .withSize(2, 2)
      .withPosition(0, 0);
    m_ChassisData = m_DriveDataTab.getLayout("Chassis Data", BuiltInLayouts.kList)
      .withSize(2, 2)
      .withPosition(2, 0);
    m_OtherData = m_DriveDataTab.getLayout("Other Data", BuiltInLayouts.kList)
      .withSize(2, 3)
      .withPosition(4, 0);
    m_OdometryData.addNumber("X", ()->{ return getPose().getTranslation().getX(); });
    m_OdometryData.addNumber("Y", ()->{ return getPose().getTranslation().getY(); });
    m_OdometryData.addNumber("Angle", ()->{ return getPose().getRotation().getDegrees(); });

    m_ChassisData.addNumber("X", ()->{ return getChassisSpeeds().vxMetersPerSecond; });
    m_ChassisData.addNumber("Y", ()->{ return getChassisSpeeds().vyMetersPerSecond; });
    m_ChassisData.addNumber("Z", ()->{ return getChassisSpeeds().omegaRadiansPerSecond; });

    m_isFieldOriented = m_OtherData.add("Field Oriented", true).getEntry();
    m_OtherData.addNumber("Gyro Rotation", ()->{ return getGyroRotation().getDegrees(); });
    m_OtherData.addNumber("Target Rotation", ()->{ return getTargetRotation().getDegrees(); });
    
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
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
   // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
   return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public Rotation2d getGyroAdjust() {
    return Rotation2d.fromDegrees(
      MathUtils.normalize(
        m_targetHeading.getDegrees() -
         MathUtils.normalize(getGyroRotation().getDegrees()))
      ).times(Constants.GYRO_ADJUST_COEFFICENT);
  }

  public void drive(double translation_x, double translation_y, double rotation) {
      // Rotating above deadband update rotation 
      // If not stay in deadband
      if(Math.abs(rotation) > Constants.Controller.Z_AXIS_DEADBAND) {
        m_targetHeading = getGyroRotation();
      }
      rotation -= getGyroAdjust().getRadians();

      m_chassisSpeeds = m_isFieldOriented.getBoolean(true) ? 
        ChassisSpeeds.fromFieldRelativeSpeeds(translation_x, translation_y, rotation, getGyroRotation()) :
        new ChassisSpeeds(translation_x, translation_y, rotation);
  }

  public void setTargetHeading(Rotation2d rotation, boolean fieldOriented) {
   if(fieldOriented) {
     m_targetHeading = rotation;
   } else {
     m_targetHeading.plus(rotation);
   }
  }

  public double getTurnRate() {
    return m_navx.getRate();
  }

  public Rotation2d getTargetRotation() {
    return m_targetHeading;
  }

  public ChassisSpeeds getChassisSpeeds(){
    return m_chassisSpeeds;
  }

  public SwerveDriveKinematics getKinematics(){
    return m_kinematics;
  }

  public Pose2d getPose() {
    return m_odometer.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
        m_odometer.resetPosition(pose, getGyroRotation());
  }

  public void updateOdometry(SwerveModuleState[] states){
    m_odometer.update(getGyroRotation(), states); 
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());

    updateOdometry(states);
}

  public void stopModules() {
        m_chassisSpeeds = new ChassisSpeeds(0, 0, 0);
  }
}
