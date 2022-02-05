package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class RotationDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private PIDController m_rotationController = new PIDController(0.5, 0.0, 0.02);
    private double m_targetAngle;

    public RotationDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
            double targetDegree) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_targetAngle = Math.toRadians(targetDegree);
        resetController();
        addRequirements(drivetrainSubsystem);
    }

    public void resetController(){
        m_rotationController.reset();
        m_rotationController.setSetpoint(m_targetAngle);
        //m_rotationController.setTolerance(0.0, 0.2);
        m_rotationController.enableContinuousInput(0.0, 2*Math.PI);
    }
    @Override
    public void initialize() {
        resetController();
    }

    @Override
    public void execute() {
        double angleDelta = m_rotationController.calculate(m_drivetrainSubsystem.getPose().getRotation().getRadians());
        SmartDashboard.putNumber("Target", m_targetAngle);
        SmartDashboard.putNumber("Delta", angleDelta);
        m_drivetrainSubsystem.setRotation(angleDelta);
    }

    @Override
    public boolean isFinished() {
      // End when the controller is at the reference.
      return m_rotationController.atSetpoint();
    }
}
