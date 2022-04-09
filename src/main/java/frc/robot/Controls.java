package frc.robot;

import beartecs.Constants;
import beartecs.controller.XboxControllerExtended;
import beartecs.math.MathUtils;
import edu.wpi.first.math.geometry.Rotation2d;

public class Controls {
  private final static XboxControllerExtended m_driverController = new XboxControllerExtended(0);
  private final static XboxControllerExtended m_operatorController = new XboxControllerExtended(1);

  // Should only be used if can't create a method
  public final static XboxControllerExtended getDriveController() {
    return m_driverController;
  }

  public final static XboxControllerExtended getOperatorController() {
    return m_operatorController;
  }

  public static boolean getResetGyroButton() {
    return m_driverController.getBackButtonPressed();
  }

  public static double getDriveForwardAxis() {
    return MathUtils.modifyAxis(m_driverController.getLeftY(), Constants.Controller.XBOX_DEADBAND);
  }

  public static double getDriveStrafeAxis() {
    return MathUtils.modifyAxis(m_driverController.getLeftX(), Constants.Controller.XBOX_DEADBAND);
  }

  public static double getDriveRotationAxis() {
    return MathUtils.modifyAxis(m_driverController.getRightX(), Constants.Controller.XBOX_DEADBAND);
  }

  public static boolean getDriveReduction() {
    return m_driverController.getRightBumperPressed();
  }

  public static Rotation2d getDrivePOV() {
    return Rotation2d.fromDegrees(m_driverController.getPOV());
  }

  public static boolean getIntakeRequestedUp() {
    return m_operatorController.getRightTriggerAxis() > 0
        || m_driverController.getRightTriggerAxis() > 0
        || m_operatorController.getRightBumperPressed();
  }

  public static boolean getIntakeRequestedDown() {
    return !getIntakeRequestedUp()
        || m_operatorController.getLeftBumperPressed()
        || m_driverController.getLeftBumperPressed();
  }

  public static boolean getIntakeForceDown() {
    return m_operatorController.getRightBumperPressed();
  }

  public static boolean getIntakeForceUp() {
    return m_operatorController.getLeftBumperPressed();
  }

  public static boolean getIntakeResetEncoderButton() {
    return m_operatorController.getStartButtonPressed();
  }

  public static double getIntakeFireSpeed() {
    return m_operatorController.getLeftTriggerAxis() > 0
        ? m_operatorController.getLeftTriggerAxis()
        : m_driverController.getLeftTriggerAxis();
  }

  public static double getIntakeSpeed() {
    return m_operatorController.getRightTriggerAxis() > 0
        ? m_operatorController.getRightTriggerAxis()
        : m_driverController.getRightTriggerAxis();
  }

}
