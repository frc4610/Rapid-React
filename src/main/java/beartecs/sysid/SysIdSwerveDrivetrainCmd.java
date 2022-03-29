package beartecs.sysid;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import beartecs.swerve.SwerveModule;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SysIdSwerveDrivetrainCmd extends CommandBase {
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private SysIdSwerveDrivetrainLogger m_logger;
  private Double m_prevAngle = 0.0;
  private Double m_prevTime = 0.0;
  private boolean m_resetComplete;

  public SysIdSwerveDrivetrainCmd(DrivetrainSubsystem drivetrainSubsystem) {

    m_drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // reset gyro and encoders
    // set timeperiod to .005
    //m_drivetrainSubsystem.setDeadband(0.0);
    // The following is called for the side-effect of resetting the 
    // drivebase odometers.
    m_drivetrainSubsystem.resetPose(m_drivetrainSubsystem.getPose());
    m_logger = new SysIdSwerveDrivetrainLogger();
    m_logger.updateThreadPriority();
    m_logger.initLogging();
    m_resetComplete = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angularPosition = m_drivetrainSubsystem.getGyroRotation().getRadians();
    double deltaAngle = angularPosition - m_prevAngle;
    double now = Timer.getFPGATimestamp();
    double deltaTime = now - m_prevTime;
    double angularRate = m_prevTime == 0 || deltaTime == 0 ? 0.0 : deltaAngle / deltaTime;
    m_prevAngle = angularPosition;
    m_prevTime = now;
    SwerveModule[] modules = m_drivetrainSubsystem.getSwerveModules();
    // Resetting encoders takes non-zero time on CAN-based encoders
    // Wait for the reset to complete
    if (!m_resetComplete) {
      for (SwerveModule mod : modules) {
        if (SysIdSwerveDrivetrainLogger.getDrivePositionMeters((WPI_TalonFX) mod.getDriveMotor()) > 0.01)
          return;
      }
      m_resetComplete = true;
    }

    m_logger.log(modules, angularPosition, angularRate);
    for (int index = 0; index < modules.length; index++) {
      modules[index].set(m_logger.getVoltage(index), 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Characterization done; disabled");
    m_drivetrainSubsystem.drive(0, 0, 0);
    m_logger.sendData();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;

  }
}