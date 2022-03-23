package swervelib.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PoseTelemetry {

  public static Field2d field = new Field2d();

  // Estimated position says where you think your robot is at
  // Based on encoders, motion, vision, etc.
  Pose2d estimatedPose = new Pose2d();

  // Actual position defines wherever the robot is actually at
  // at any time. It is unknowable in real life. The simulation
  // generates this as its primary output.
  Pose2d actualPose = new Pose2d();

  // Swerve Module positions
  Pose2d[] modulePoses = { null, null, null, null };

  Trajectory trajectory = null;

  public PoseTelemetry() {
    SmartDashboard.putData("Field", field);
  }

  public void setActualPose(Pose2d act) {
    actualPose = act;
  }

  public void setEstimatedPose(Pose2d est) {
    estimatedPose = est;
  }

  public void setSwerveModulePoses(Pose2d[] mods) {
    modulePoses = mods;
  }

  public void setTrajectory(Trajectory traj) {
    trajectory = traj;
  }

  public void update() {
    field.setRobotPose(actualPose);
    field.getObject("EstPose").setPose(estimatedPose);
    field.getObject("ModPoses").setPoses(modulePoses);
    field.getObject("Trajectory").setTrajectory(trajectory);
  }
}