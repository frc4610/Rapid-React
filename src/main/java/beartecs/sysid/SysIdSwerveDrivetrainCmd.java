package beartecs.sysid;

import beartecs.Logging.RobotLogger;
import beartecs.swerve.SwerveModule;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SysIdSwerveDrivetrainCmd extends CommandBase {
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private SysIdSwerveDrivetrainLogger m_sysIdLogger;
  private RobotLogger m_logger = RobotContainer.getLogger();
  private Double m_prevAngle = 0.0;
  private Double m_prevTime = 0.0;

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
    m_drivetrainSubsystem.zeroGyro();
    m_drivetrainSubsystem.resetPose(m_drivetrainSubsystem.getPose());

    m_sysIdLogger = new SysIdSwerveDrivetrainLogger();
    m_sysIdLogger.updateThreadPriority();
    m_sysIdLogger.initLogging();
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

    m_sysIdLogger.log(modules, angularPosition, angularRate);
    for (int index = 0; index < modules.length; index++) {
      modules[index].set(m_sysIdLogger.m_motorVoltage, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_logger.logInfo("Characterization done; disabled");
    m_drivetrainSubsystem.drive(0, 0, 0);
    m_sysIdLogger.sendData();
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