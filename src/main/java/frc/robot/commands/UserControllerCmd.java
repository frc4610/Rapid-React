package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.*;
import frc.robot.subsystems.DrivetrainSubsystem;
import globals.utils.math.MathUtils;
import frc.robot.RobotContainer;

public class UserControllerCmd extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(5);
    private ProfiledPIDController m_rotationController = Auto.PID_THETA.getProfiledPidController();

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
        final double xSpeed = -m_xSpeedLimiter.calculate(RobotContainer.getDriveForwardAxis())
                * Motor.MAX_VELOCITY_MPS;

        final double ySpeed = -m_ySpeedLimiter.calculate(RobotContainer.getDriveStrafeAxis())
                * Motor.MAX_VELOCITY_MPS;

        double rot = -m_rotLimiter.calculate(RobotContainer.getDriveRotationAxis())
                * Motor.MAX_ANGULAR_VELOCITY_RPS;

        double drivePOV = MathUtils.angleWrap(RobotContainer.getDrivePOV().getDegrees());
        if (drivePOV != -1) { // Pressing one of the POV keys
            double rotationOutput = m_rotationController
                    .calculate(
                            m_drivetrainSubsystem.getGyroRotation().getRadians(),
                            Math.toRadians(drivePOV));

            if (Math.abs(rotationOutput) < Controller.XBOX_DEADBAND) {
                rotationOutput = 0.0;
            } else {
                rot = rotationOutput;
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
