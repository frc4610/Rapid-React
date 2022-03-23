package swervelib;

public final class Constants {
  // TODO: Add smart dashboard button to log data from calibration mode
  // Set calibration mode to true then move all wheels straight and you'll get offsets
  public final static boolean CALIBRATION_MODE = false;
  // Will fix some issues with abs positions when abs pos fails
  public final static boolean ENABLE_ABS_ENCODER_POS_ERROR_CHECKS = true;
  public final static int ABS_ENCODER_ERROR_RETRY_COUNT = 3;

  public final static boolean ALIGN_RANGE_ENABLE = true;
  public final static boolean BOOT_TO_ABS = true;

  public final class Sim {
    static public final int STATUS_FRAME_PERIOD_MS = 20; // Falcons update every 250 ms but in sim we want it to update faster
    static public final double SAMPLE_RATE_SEC = 0.02;
  }
}
