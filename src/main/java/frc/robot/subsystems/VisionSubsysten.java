package frc.robot.subsystems;

import java.util.OptionalDouble;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Limelight;
import frc.robot.utils.MathUtils;

public class VisionSubsysten extends SubsystemBase {

  private static final double TARGET_ALLOWABLE_ERROR = Math.toRadians(2.5);

  private final Limelight m_LimeLight = new Limelight();

  private final DrivetrainSubsystem m_DrivetrainSubsystem;

  private double xFromTarget = Double.NaN;
  private double yFromTarget = Double.NaN;
  
  private final NetworkTableEntry distanceToTargetEntry;
  private final NetworkTableEntry dXOuterEntry;
  private final NetworkTableEntry dYOuterEntry;
  private final NetworkTableEntry canSeeInnerTargetEntry;

  private boolean hasTarget;
  private boolean isInnerTargetVisible;
  private OptionalDouble distanceToTarget = OptionalDouble.empty();
  private OptionalDouble angleToTarget = OptionalDouble.empty();

  public VisionSubsysten(DrivetrainSubsystem drivetrainSubsystem) {
    m_DrivetrainSubsystem = drivetrainSubsystem;
    ShuffleboardTab tab = Shuffleboard.getTab("Vision");
    distanceToTargetEntry = tab.add("distance to target", 0.0)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
    dXOuterEntry = tab.add("dXOuter", 0.0)
        .withPosition(1, 0)
        .withSize(1, 1)
        .getEntry();
    dYOuterEntry = tab.add("dYOuter", 0.0)
        .withPosition(2, 0)
        .withSize(1, 1)
        .getEntry();
    canSeeInnerTargetEntry = tab.add("can see inner target", false)
        .withPosition(3, 0)
        .withSize(1, 1)
        .getEntry();
    tab.addNumber("target angle", () -> Math.toDegrees(getAngleToTarget().orElse(Double.NaN)))
        .withPosition(4, 0)
        .withSize(1, 1);
    tab.addBoolean("Is on target", this::isOnTarget)
        .withPosition(5, 0)
        .withSize(1, 1);
    tab.addNumber("Horizontal Target Error", () -> {
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
    return distanceToTarget;
  }

  public OptionalDouble getAngleToTarget() {
    return angleToTarget;
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
    return hasTarget;
  }

  public boolean isInnerTargetVisible() {
    return isInnerTargetVisible;
  }

  public Vector2d getPredictedPostition() {
    Vector2d position = new Vector2d(xFromTarget, yFromTarget);
    return position;
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
    // This method will be called once per scheduler run

  }
}