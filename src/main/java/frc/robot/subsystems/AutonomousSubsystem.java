package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AutonomousSubsystem extends SubsystemBase {

  private final SendableChooser<Trajectory> m_autoChooser = new SendableChooser<Trajectory>();
  private TrajectoryConfig m_config;

  public AutonomousSubsystem(DrivetrainSubsystem driveSubsystem) {
    m_config = new TrajectoryConfig(
        Constants.Autonomous.MAX_VELOCITY_METERS_PER_SECOND,
        Constants.Autonomous.MAX_ACCELERATION_METERS_PER_SECOND)
            .setKinematics(driveSubsystem.getKinematics());

    final String trajectoryPathName = "Paths"; // Grrr String literals
    // Parallel Code Generation
    try {
      File directory = Filesystem.getDeployDirectory();
      for (String trajectoryJSON : directory.list()) {
        if (!trajectoryJSON.contains(".json"))
          continue;
        Path trajectoryPath = directory.toPath().resolve(trajectoryPathName + trajectoryJSON);
        m_autoChooser.addOption(trajectoryJSON, TrajectoryUtil.fromPathweaverJson(trajectoryPath));
      }
    } catch (IOException ex) {
      DriverStation.reportError("Unable to load trajectorys: ", ex.getStackTrace());
    }

    m_autoChooser.addOption("S Curve", TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        m_config));

    m_autoChooser.setDefaultOption("2 X", TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(1, 0)),
        new Pose2d(2, 0, new Rotation2d(0)),
        m_config));

    SmartDashboard.putData("Auto Selector", m_autoChooser);
  }

  // std::optional?
  public Trajectory getTrajectory() {
    return m_autoChooser.getSelected();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }

}