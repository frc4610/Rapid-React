package frc.robot.utils;

import edu.wpi.first.wpilibj.I2C;

public class ArduinoI2C extends I2C {
  public ArduinoI2C(I2C.Port port, int deviceAddress) {
    super(port, deviceAddress);
  }

  public String formatString(String key, String val) {
    return "\"" + key + "\"" + "\"" + val + "\"";
  }

  public boolean sendString(String text) {
    char[] array = text.toCharArray();

    byte[] data = new byte[array.length];
    for (int i = 0; i < array.length; i++) {
      data[i] = (byte) array[i];
    }

    // Todo add byte buffer for receiving
    return transaction(data, data.length, null, 0);
  }

  public boolean sendInt(String key, int val) {
    return sendString(formatString(key, Integer.toHexString(val)));
  }
}
