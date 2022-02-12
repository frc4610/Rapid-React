package frc.robot.subsystems;

import java.util.OptionalDouble;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Limelight;
import frc.robot.utils.MathUtils;

public class VisionSubsysten extends SubsystemBase {

  private static final double TARGET_ALLOWABLE_ERROR = Math.toRadians(2.5);

  private final Limelight m_LimeLight = new Limelight();

  private final DrivetrainSubsystem m_DrivetrainSubsystem;

  private final NetworkTableEntry m_distanceToTargetEntry;
  private final NetworkTableEntry m_XEntry;
  private final NetworkTableEntry m_YEntry;

  private boolean m_hasTarget;
  private OptionalDouble m_distanceToTarget = OptionalDouble.empty();
  private OptionalDouble m_angleToTarget = OptionalDouble.empty();

  private ShuffleboardTab m_VisionTab;

  public VisionSubsysten(DrivetrainSubsystem drivetrainSubsystem) {
    m_DrivetrainSubsystem = drivetrainSubsystem;
    m_VisionTab = Shuffleboard.getTab("Vision");
    m_distanceToTargetEntry = m_VisionTab.add("distance to target", 0.0)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
    m_XEntry = m_VisionTab.add("X", 0.0)
        .withPosition(1, 0)
        .withSize(1, 1)
        .getEntry();
    m_YEntry = m_VisionTab.add("Y", 0.0)
        .withPosition(2, 0)
        .withSize(1, 1)
        .getEntry();
    m_VisionTab.addNumber("target angle", () -> Math.toDegrees(getAngleToTarget().orElse(Double.NaN)))
        .withPosition(4, 0)
        .withSize(1, 1);
    m_VisionTab.addBoolean("Is on target", this::isOnTarget)
        .withPosition(5, 0)
        .withSize(1, 1);
    m_VisionTab.addBoolean("has target", this::hasTarget)
        .withPosition(6, 0)
        .withSize(1, 1);
    m_VisionTab.addNumber("Horizontal Target Error", () -> {
      double gyroAngle = m_DrivetrainSubsystem.getPose().getRotation().getRadians();
      return getDistanceToTarget().orElse(0.0) *
          (Math.sin(gyroAngle - getAngleToTarget().orElse(0.0)) / Math.sin(Math.PI / 2.0 - gyroAngle));
    })
        .withPosition(6, 0)
        .withSize(1, 1);
  }

  public void setCamMode(Limelight.CamMode mode) {
    m_LimeLight.setCamMode(mode);
  }

  public OptionalDouble getDistanceToTarget() {
    return m_distanceToTarget;
  }

  public OptionalDouble getAngleToTarget() {
    return m_angleToTarget;
  }

  public OptionalDouble getHorizontalError() {
    OptionalDouble distanceToTargetOpt = getDistanceToTarget();
    OptionalDouble angleToTargetOpt = getAngleToTarget();

    if (distanceToTargetOpt.isEmpty() || angleToTargetOpt.isEmpty()) {
      return OptionalDouble.empty();
    }

    double gyroAngle = m_DrivetrainSubsystem.getPose().getRotation().getRadians();
    return OptionalDouble.of(distanceToTargetOpt.getAsDouble() *
        (Math.sin(gyroAngle - angleToTargetOpt.getAsDouble()) / Math.sin(Math.PI / 2.0 - gyroAngle)));
  }

  public boolean hasTarget() {
    return m_hasTarget;
  }

  public boolean isOnTarget() {
    OptionalDouble targetAngle = getAngleToTarget();
    if (targetAngle.isEmpty()) {
      return false;
    }

    double delta = targetAngle.getAsDouble() - m_DrivetrainSubsystem.getPose().getRotation().getRadians();
    if (delta > Math.PI) {
      delta = 2.0 * Math.PI - delta;
    }

    return MathUtils.epsilonEquals(
        delta,
        0,
        TARGET_ALLOWABLE_ERROR);
  }

  public void setLedMode(Limelight.LedMode mode) {
    m_LimeLight.setLedMode(mode);
  }

  public void setSnapshotEnabled(boolean isEnabled) {
    m_LimeLight.setSnapshotsEnabled(isEnabled);
  }

  @Override
  public void periodic() {
    // Shooter limelight
    // Determine whether the Limelight has a target or not
    m_hasTarget = m_LimeLight.hasTarget();
    if (m_hasTarget) {
      // Calculate the distance to the outer target
      Vector2d targetPosition = m_LimeLight.getTargetPosition();
      double distanceToTarget = m_LimeLight.getDistance(
          Constants.Limelight.LIMELIGHT_HEIGHT,
          Constants.Limelight.TARGET_HEIGHT,
          Constants.Limelight.LIMELIGHT_ANGLE);

      // Get the field oriented angle for the outer target, with latency compensation
      double angleToTarget = m_DrivetrainSubsystem.getLagCompPose(Timer.getFPGATimestamp() -
          m_LimeLight.getPipelineLatency() / 1000.0).getRotation().getRadians() -
          targetPosition.x;
      m_XEntry.setDouble(distanceToTarget * Math.sin(angleToTarget));
      m_YEntry.setDouble(distanceToTarget * Math.cos(angleToTarget));

      m_distanceToTarget = OptionalDouble.of(distanceToTarget);
      m_angleToTarget = OptionalDouble.of(angleToTarget);
    } else {
      m_distanceToTarget = OptionalDouble.empty();
      m_angleToTarget = OptionalDouble.empty();
    }
    // Update shuffleboard
    m_distanceToTargetEntry.setDouble(m_distanceToTarget.orElse(-1.0));
  }
}