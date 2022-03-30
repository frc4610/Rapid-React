package beartecs.CAN;

public class CANConfig {
  public final Integer deviceNumber;
  public final String canBus;

  public CANConfig(Integer deviceNumber) {
    this(deviceNumber, "rio");
  }

  public CANConfig(Integer deviceNumber, String canBus) {
    this.deviceNumber = deviceNumber;
    this.canBus = canBus;
  }
}
