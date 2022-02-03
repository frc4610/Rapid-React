package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class AutonomousDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_XAxis;
    private final DoubleSupplier m_YAxis;
    private final DoubleSupplier m_ZAxis;

    public AutonomousDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
        DoubleSupplier XAxis,
        DoubleSupplier YAxis,
        DoubleSupplier ZAxis) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_XAxis = XAxis;
        this.m_YAxis = YAxis;
        this.m_ZAxis = ZAxis;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_XAxis.getAsDouble(),
                        m_YAxis.getAsDouble(),
                        m_ZAxis.getAsDouble(),
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
