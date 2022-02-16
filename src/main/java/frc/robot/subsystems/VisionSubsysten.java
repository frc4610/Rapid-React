package frc.robot.subsystems;

import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Limelight;
import frc.robot.utils.MathUtils;

public class VisionSubsysten extends SubsystemBase {

  private final Limelight m_LimeLight = new Limelight();

  private final DrivetrainSubsystem m_drivetrainSubsystem;

  private final NetworkTableEntry m_distanceToTargetEntry;
  private final NetworkTableEntry m_XEntry;
  private final NetworkTableEntry m_YEntry;

  private boolean m_hasTarget;
  private OptionalDouble m_distanceToTarget = OptionalDouble.empty();
  private OptionalDouble m_angleToTarget = OptionalDouble.empty();

  private ShuffleboardTab m_visionTab;
  private ShuffleboardLayout m_visionLayout;
  private ShuffleboardLayout m_limelightLayout;

  public VisionSubsysten(DrivetrainSubsystem drivetrainSubsystem) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_visionTab = Shuffleboard.getTab("Vision");
    m_visionLayout = m_visionTab.getLayout("Vision Data", BuiltInLayouts.kGrid)
        .withSize(4, 2)
        .withPosition(0, 0);
    m_XEntry = m_visionLayout.add("X", 0.0)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
    m_YEntry = m_visionLayout.add("Y", 0.0)
        .withPosition(1, 0)
        .withSize(1, 1)
        .getEntry();
    m_visionLayout.addBoolean("on target", this::isOnTarget)
        .withPosition(3, 0)
        .withSize(1, 1);
    m_visionLayout.addBoolean("has target", this::hasTarget)
        .withPosition(4, 0)
        .withSize(1, 1);
    m_distanceToTargetEntry = m_visionLayout.add("distance to target", 0.0)
        .withPosition(0, 1)
        .withSize(1, 1)
        .getEntry();
    m_visionLayout.addNumber("target angle", () -> Math.toDegrees(m_angleToTarget.orElse(Double.NaN)))
        .withPosition(1, 1)
        .withSize(1, 1);
    m_visionLayout.addNumber("target error", () -> getHorizontalError().getAsDouble())
        .withPosition(1, 2)
        .withSize(1, 1);
    m_limelightLayout = m_visionTab.getLayout("Limelight Data", BuiltInLayouts.kGrid)
        .withSize(2, 1)
        .withPosition(0, 2);
    m_limelightLayout.addNumber("X", () -> m_LimeLight.getTargetPosition().x)
        .withPosition(0, 0)
        .withSize(1, 1);
    m_limelightLayout.addNumber("Y", () -> m_LimeLight.getTargetPosition().y)
        .withPosition(1, 0)
        .withSize(1, 1);
    m_limelightLayout.addNumber("Skew", () -> m_LimeLight.getTargetSkew())
        .withPosition(2, 0)
        .withSize(1, 1);
  }

  public void setCamMode(Limelight.CamMode mode) {
    m_LimeLight.setCamMode(mode);
  }

  public OptionalDouble getDistanceToTarget() {
    return m_distanceToTarget;
  }

  public Rotation2d getRotationToTarget() {
    return Rotation2d.fromDegrees(m_angleToTarget.getAsDouble());
  }

  public Rotation2d getRotationToTargetCompensated() {
    return Rotation2d.fromDegrees(m_angleToTarget.getAsDouble());
  }

  public OptionalDouble getHorizontalError() {
    if (m_distanceToTarget.isEmpty() || m_angleToTarget.isEmpty()) {
      return OptionalDouble.empty();
    }

    double gyroAngle = m_drivetrainSubsystem.getPose().getRotation().getRadians();
    return OptionalDouble.of(m_distanceToTarget.getAsDouble() *
        (Math.sin(gyroAngle - m_angleToTarget.getAsDouble()) / Math.sin(Math.PI / 2.0 - gyroAngle)));
  }

  public boolean hasTarget() {
    return m_hasTarget;
  }

  public boolean isOnTarget() {
    if (m_angleToTarget.isEmpty()) {
      return false;
    }

    double delta = m_angleToTarget.getAsDouble() - m_drivetrainSubsystem.getPose().getRotation().getRadians();
    if (delta > Math.PI) {
      delta = 2.0 * Math.PI - delta;
    }

    return MathUtils.epsilonEquals(
        delta,
        0,
        Constants.Limelight.TARGET_ALLOWABLE_ERROR);
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
      double distanceToTarget = m_LimeLight.getDistance(
          Constants.Limelight.LIMELIGHT_HEIGHT,
          Constants.Limelight.TARGET_HEIGHT,
          Constants.Limelight.LIMELIGHT_ANGLE);

      double angleToTarget = m_drivetrainSubsystem.getLagCompPose(Timer.getFPGATimestamp() -
          m_LimeLight.getPipelineLatency() / 1000.0).getRotation().getRadians() -
          Rotation2d.fromDegrees(m_LimeLight.getTargetPosition().x).getRadians();

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