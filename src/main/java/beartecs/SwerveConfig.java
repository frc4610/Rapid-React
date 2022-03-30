package beartecs;

public class SwerveConfig {
  public int DRIVE_MOTOR_DEVICE_NUMBER;
  public int STEER_MOTOR_DEVICE_NUMBER;
  public int CAN_CODER_DEVICE_NUMBER;
  public double CAN_CODER_OFFSET;

  public SwerveConfig(int drive_motor, int steer_motor, int steer_encoder, double wheel_offset) {
    DRIVE_MOTOR_DEVICE_NUMBER = drive_motor;
    STEER_MOTOR_DEVICE_NUMBER = steer_motor;
    CAN_CODER_DEVICE_NUMBER = steer_encoder;
    CAN_CODER_OFFSET = wheel_offset;
  }
}