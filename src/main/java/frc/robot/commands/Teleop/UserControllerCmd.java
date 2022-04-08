package frc.robot.commands.Teleop;

import beartecs.Constants.*;
import beartecs.math.MathUtils;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.RobotContainer;

public class UserControllerCmd extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(5);
    private ProfiledPIDController m_rotationController = Auto.PID_THETA.getProfiledPidController();
    private double m_speedModifier = 1.0;

    public UserControllerCmd(DrivetrainSubsystem drivetrainSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
        double radianTolerance = Math.toRadians(Motor.TURN_TOLERANCE);
        m_rotationController.setTolerance(radianTolerance, radianTolerance);
        m_rotationController.reset(drivetrainSubsystem.getGyroRotation().getRadians());
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        if (RobotContainer.getDriveReduction()) {
            m_speedModifier = 0.2;
        } else {
            m_speedModifier = 1;
        }
        final double xSpeed = -m_xSpeedLimiter.calculate(RobotContainer.getDriveForwardAxis())
                * Motor.MAX_VELOCITY_MPS * m_speedModifier;

        final double ySpeed = -m_ySpeedLimiter.calculate(RobotContainer.getDriveStrafeAxis())
                * Motor.MAX_VELOCITY_MPS * m_speedModifier;

        double rot = -m_rotLimiter.calculate(RobotContainer.getDriveRotationAxis())
                * Motor.MAX_ANGULAR_VELOCITY_RPS * m_speedModifier;

        if (RobotContainer.getDrivePOV().getDegrees() != -1) { // Pressing one of the POV keys
            double rotationOutput = m_rotationController
                    .calculate(
                            m_drivetrainSubsystem.getGyroRotation().getRadians(),
                            Math.toRadians(-MathUtils.angleWrap(RobotContainer.getDrivePOV().getDegrees())));

            if (Math.abs(rotationOutput) < Controller.XBOX_DEADBAND) {
                rotationOutput = 0.0;
            } else {
                rot = rotationOutput * Motor.MAX_ANGULAR_VELOCITY_RPS;
            }
        } else {
            m_rotationController.reset(m_drivetrainSubsystem.getGyroRotation().getRadians());
        }
        m_drivetrainSubsystem.drive(xSpeed, ySpeed, rot);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.stopModules();
    }
}
