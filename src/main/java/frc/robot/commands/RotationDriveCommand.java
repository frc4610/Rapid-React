package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

//getGyroAdjustment to fix sway

public class RotationDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private double m_targetAngle;

    ProfiledPIDController m_rotationController =
      new ProfiledPIDController(0.2, 0, 0, new Constraints(
          DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
          DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 2.0)
    );
    
    public void resetController(){
        m_rotationController.reset(m_targetAngle);
        m_rotationController.setGoal(m_targetAngle);
        m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public RotationDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
            double targetDegree) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_targetAngle = Math.toRadians(targetDegree);
        resetController();
        addRequirements(drivetrainSubsystem);
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
        //m_drivetrainSubsystem.setRotation(angleDelta);
    }

    @Override
    public boolean isFinished() {
      // End when the controller is at the reference.
      return m_rotationController.atSetpoint();
    }
}
