package frc.robot.commands;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import swervelib.SwerveModule;

public class PlayMidiCommand extends CommandBase {

  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final Orchestra m_orchestra;
  private int m_timeToPlayLoops = 0;
  private boolean m_hasPlayed = false;

  public PlayMidiCommand(DrivetrainSubsystem subsystem, String song) {
    m_drivetrainSubsystem = subsystem;

    ArrayList<TalonFX> instruments = new ArrayList<TalonFX>();

    SwerveModule[] swerves = m_drivetrainSubsystem.getSwerveModules();
    for (int i = 0; i < swerves.length; ++i) {
      instruments.add((TalonFX) swerves[i].getDriveMotor());
      instruments.add((TalonFX) swerves[i].getSteerMotor());
    }
    m_orchestra = new Orchestra(instruments, song);
    m_timeToPlayLoops = 10;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    if (m_orchestra.isPlaying())
      m_hasPlayed = true;
    if (m_timeToPlayLoops > 0) {
      --m_timeToPlayLoops;
      if (m_timeToPlayLoops == 0) {
        m_orchestra.play();
      }
    }
  }

  @Override
  public boolean isFinished() {
    return m_hasPlayed && !m_orchestra.isPlaying();
  }

}