package frc.robot.subsystems;

import java.io.File;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.utils.BaseSubsystem;

public class AutonomousSubsystem extends BaseSubsystem {

  private final SendableChooser<Trajectory> m_autoChooser = new SendableChooser<Trajectory>();
  private TrajectoryConfig m_config;

  public AutonomousSubsystem(DrivetrainSubsystem driveSubsystem) {
    m_config = new TrajectoryConfig(
        Autonomous.MAX_VELOCITY_METERS_PER_SECOND,
        Autonomous.MAX_ACCELERATION_METERS_PER_SECOND)
        .setKinematics(driveSubsystem.getKinematics());

    File pathplannerDir = null;
    File[] deployDir = Filesystem.getDeployDirectory().listFiles();
    for (File file : deployDir) {
      if (!file.isDirectory())
        continue;
      if (file.getName().contains("pathplanner")) {
        pathplannerDir = file;
      }
    }
    for (File file : pathplannerDir.listFiles()) {
      if (!file.isFile())
        continue;
      int pathSuffixIndex = file.getName().indexOf(".path");
      loadPathPlanner(file.getName().substring(0, pathSuffixIndex));
    }
    loadInternalTrajectories();
    SmartDashboard.putData("Auto Selector", m_autoChooser);
  }

  public void loadInternalTrajectories() {
    m_autoChooser.addOption("S Curve", TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        m_config));

    Trajectory defaultTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(2, 0)),
        new Pose2d(4, 0, new Rotation2d(0)),
        m_config);

    m_autoChooser.addOption("2 X", defaultTrajectory);
    m_autoChooser.setDefaultOption("2 X", defaultTrajectory);
  }

  public void loadPathPlanner(String name) {
    m_autoChooser.addOption(name, PathPlanner.loadPath(name, Autonomous.MAX_VELOCITY_METERS_PER_SECOND,
        Autonomous.MAX_ACCELERATION_METERS_PER_SECOND));
  }

  public Optional<Trajectory> getTrajectory() {
    return Optional.ofNullable(m_autoChooser.getSelected());
  }

  public void showCurrentTrajectory(Trajectory trajectory) {
    RobotContainer.dashboardField.getObject("Trajectory").setTrajectory(trajectory);
  }
}