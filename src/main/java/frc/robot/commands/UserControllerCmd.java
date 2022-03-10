package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.*;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class UserControllerCmd extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    public UserControllerCmd(DrivetrainSubsystem drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        final double xSpeed = -m_xSpeedLimiter.calculate(m_translationXSupplier.getAsDouble())
                * Motor.MAX_VELOCITY_MPS;

        final double ySpeed = -m_ySpeedLimiter.calculate(m_translationYSupplier.getAsDouble())
                * Motor.MAX_VELOCITY_MPS;

        final double rot = -m_rotLimiter.calculate(m_rotationSupplier.getAsDouble())
                * Motor.MAX_ANGULAR_VELOCITY_RPS;
        m_drivetrainSubsystem.drive(xSpeed, ySpeed, rot);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.stopModules();
    }
}
