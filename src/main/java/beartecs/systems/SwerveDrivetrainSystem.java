package beartecs.systems;

import java.util.ArrayList;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import beartecs.Constants;
import beartecs.Constants.*;
import beartecs.swerve.*;
import beartecs.swerve.sim.*;

public class SwerveDrivetrainSystem {
  QuadSwerveSim swerveDt;
  ArrayList<SwerveModule> realModules = new ArrayList<SwerveModule>(QuadSwerveSim.NUM_MODULES);
  ArrayList<SwerveModuleSim> simModules = new ArrayList<SwerveModuleSim>(QuadSwerveSim.NUM_MODULES);

  ArrayList<SteerController> steerMotorControllers = new ArrayList<SteerController>(QuadSwerveSim.NUM_MODULES);
  ArrayList<DriveController> driveMotorControllers = new ArrayList<DriveController>(QuadSwerveSim.NUM_MODULES);
  ArrayList<AbsoluteEncoder> steerEncoders = new ArrayList<AbsoluteEncoder>(QuadSwerveSim.NUM_MODULES);

  Gyroscope gyro;

  Field2d field;
  Pose2d endPose;
  PoseTelemetry dtPoseView;

  SwerveDrivePoseEstimator m_poseEstimator;
  Pose2d curPose = new Pose2d();
  Pose2d fieldPose = new Pose2d(); // Field-referenced orign
  double curSpeed = 0;
  SwerveModuleState[] states;
  PIDController xyController = Auto.PID_XY.getPidController();
  ProfiledPIDController thetaController = Auto.PID_THETA.getProfiledPidController();

  public SwerveDrivetrainSystem(ArrayList<SwerveModule> realModules, Gyroscope gyro, ModuleConfiguration modConfig) {
    this.gyro = gyro;
    this.realModules = realModules;

    if (RobotBase.isSimulation()) {
      simModules.add(modConfig.createSim(realModules.get(0), "FL"));
      simModules.add(modConfig.createSim(realModules.get(1), "FR"));
      simModules.add(modConfig.createSim(realModules.get(2), "BL"));
      simModules.add(modConfig.createSim(realModules.get(3), "BR"));
    }

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    field = PoseTelemetry.field;
    field.setRobotPose(curPose);
    endPose = curPose;
    dtPoseView = new PoseTelemetry();

    swerveDt = new QuadSwerveSim(Constants.SWERVE_CONFIG.TRACKWIDTH_METERS,
        Constants.SWERVE_CONFIG.TRACKWIDTH_METERS,
        Constants.SWERVE_CONFIG.MASS,
        Constants.SWERVE_CONFIG.INERTIA,
        simModules);

    // Trustworthiness of the internal model of how motors should be moving
    // Measured in expected standard deviation (meters of position and degrees of
    // rotation)
    var stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(2));

    // Trustworthiness of gyro in radians of standard deviation.
    var localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.1));

    // Trustworthiness of the vision system
    // Measured in expected standard deviation (meters of position and degrees of
    // rotation)
    var visionMeasurementStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));

    m_poseEstimator = new SwerveDrivePoseEstimator(getGyroRotation(), curPose,
        Constants.SWERVE_CONFIG.KINEMATICS, stateStdDevs, localMeasurementStdDevs, visionMeasurementStdDevs,
        Sim.SAMPLE_RATE_SEC);

    setKnownPose(curPose);
  }

  /**
   * Handles discontinuous jumps in robot pose. Used at the start of
   * autonomous, if the user manually drags the robot across the field in the
   * Field2d widget, or something similar to that.
   * @param pose
   */
  public void modelReset(Pose2d pose) {
    field.setRobotPose(pose);
    swerveDt.modelReset(pose);
    resetPose(pose);
  }

  /**
   * Advance the simulation forward by one step
   * @param isDisabled
   * @param batteryVoltage
   */
  public void update(boolean isDisabled, double batteryVoltage) {

    // Check if the user moved the robot with the Field2D
    // widget, and reset the model if so.
    Pose2d startPose = field.getRobotPose();
    Transform2d deltaPose = startPose.minus(endPose);
    if (deltaPose.getRotation().getDegrees() > 0.01 || deltaPose.getTranslation().getNorm() > 0.01) {
      modelReset(startPose);
    }

    // Calculate and update input voltages to each motor.
    if (isDisabled) {
      for (int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++) {
        simModules.get(idx).setInputVoltages(0.0, 0.0);
      }
    } else {
      for (int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++) {
        double steerVolts = realModules.get(idx).getSteerController().getOutputVoltage();
        double wheelVolts = realModules.get(idx).getDriveController().getOutputVoltage();
        simModules.get(idx).setInputVoltages(wheelVolts, steerVolts);
      }
    }

    //Update the main drivetrain plant model
    swerveDt.update(Sim.SAMPLE_RATE_SEC);
    endPose = swerveDt.getPose();

    // Update each encoder
    for (int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++) {
      double azmthShaftPos = simModules.get(idx).getAzimuthEncoderPositionRev();
      double steerMotorPos = simModules.get(idx).getAzimuthMotorPositionRev();
      double wheelPos = simModules.get(idx).getWheelEncoderPositionRev();

      double azmthShaftVel = simModules.get(idx).getAzimuthEncoderVelocityRPM();
      double steerVelocity = simModules.get(idx).getAzimuthMotorVelocityRPM();
      double wheelVelocity = simModules.get(idx).getWheelEncoderVelocityRPM();

      realModules.get(idx).getSteerEncoder().setAbsoluteEncoder(azmthShaftPos, azmthShaftVel);
      realModules.get(idx).getSteerController().setSteerEncoder(steerMotorPos, steerVelocity);
      realModules.get(idx).getDriveController().setDriveEncoder(wheelPos, wheelVelocity);
    }

    // Update associated devices based on drivetrain motion
    field.setRobotPose(endPose);
    gyro.setAngle(swerveDt.getPose().getRotation().getDegrees());

    // Based on gyro and measured module speeds and positions, estimate where our
    // robot should have moved to.
    Pose2d prevPose = curPose;
    if (states != null) {
      curPose = m_poseEstimator.update(getGyroRotation(), states[0], states[1], states[2], states[3]);

      // Calculate a "speedometer" velocity in ft/sec
      Transform2d chngPose = new Transform2d(prevPose, curPose);
      curSpeed = Units.metersToFeet(chngPose.getTranslation().getNorm()) / Sim.SAMPLE_RATE_SEC;
    }
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    this.states = desiredStates;
  }

  public SwerveModuleState[] getModuleStates() {
    return states;
  }

  public Pose2d getPose() {
    return field.getRobotObject().getPose();
  }

  public void resetPose(Pose2d pose) {
    modelReset(pose);
  }

  public void setKnownPose(Pose2d in) {
    resetWheelEncoders();
    // No need to reset gyro, pose estimator does that.
    m_poseEstimator.resetPosition(in, getGyroRotation());
    curPose = in;
  }

  public void zeroGyro() {
    gyro.zeroGyro();
  }

  public Rotation2d getGyroRotation() {
    return gyro.getGyroRotation();
  }

  public void updateTelemetry() {
    if (RobotBase.isSimulation()) {
      dtPoseView.setActualPose(getPose());
    }
    dtPoseView.setEstimatedPose(curPose);
  }

  public void resetWheelEncoders() {
    for (int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++) {
      realModules.get(idx).resetDriveEncoder();
    }
  }

  public Command createCommandForTrajectory(PathPlannerTrajectory trajectory, BaseSubsystem driveSubsystem) {
    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
        trajectory,
        () -> getPose(), // Functional interface to feed supplier
        Constants.SWERVE_CONFIG.KINEMATICS,
        // Position controllers
        xyController,
        xyController,
        thetaController,
        commandStates -> setModuleStates(commandStates),
        driveSubsystem);
    return swerveControllerCommand;
  }
}