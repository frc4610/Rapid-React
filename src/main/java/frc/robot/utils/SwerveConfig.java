package frc.robot.utils;

public class SwerveConfig {
  public int DRIVE_MOTOR;
  public int STEER_MOTOR;
  public int STEER_ENCODER;
  public double STEER_OFFSET;

  public SwerveConfig(int drive_motor, int steer_motor, int steer_encoder, double wheel_offset) {
    DRIVE_MOTOR = drive_motor;
    STEER_MOTOR = steer_motor;
    STEER_ENCODER = steer_encoder;
    STEER_OFFSET = wheel_offset;
  }
}