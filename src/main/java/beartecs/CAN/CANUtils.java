package beartecs.CAN;

// 29 bit system
// Device: 6
// Manufacturer: 8
// API Class: 6
// API Index: 4
// Device #: 6
public class CANUtils {
  public enum Device {
    BROADCAST_MESSAGE(0),
    ROBOT_CONTROLLER(1),
    MOTOR_CONTROLLER(2),
    RELAY_CONTROLLER(3),
    GYRO_SENSOR(4),
    ACCELEROMETER(5),
    ULTRASONIC_SENSOR(6),
    GEAR_TOOTH_SENSOR(7),
    POWER_DISTRIBUTION_MODULE(8),
    PNEUMATICS(9),
    MISC(10),
    IO_BREAKOUT(11),
    RESERVED(12), // 12 - 30
    FIRMWARE_UPDATE(31);

    private Integer id;

    Device(int _id) {
      id = _id;
    }

    public int getId() {
      return this.id;
    }
  }

  public enum Manufacturer {
    BROADCAST(0),
    NI(1),
    LUMINARY(2),
    DEKA(3),
    CTR_ELECTRONICS(4),
    REV_ROBOTICS(5),
    GRAPPLE(6),
    MINDSENSORS(7),
    TEAM(8),
    KAUAI_LABS(9),
    COPPERFORGE(10),
    FUSION(11),
    STUDICA(12),
    RESERVED(13); // 13 - 255

    private Integer id;

    Manufacturer(int _id) {
      id = _id;
    }

    public int getId() {
      return this.id;
    }
  }

  public enum API_CLASS {
    VOLTAGE_CONTROL(0),
    SPEED_CONTROL(1),
    VOLTAGE_COMPENSATION(2),
    POSITION_CONTROL(3),
    CURRENT_CONTROL(4),
    STATUS(5),
    PERIODIC_STATUS(6),
    CONFIGURATION(7),
    ACK(8);

    private Integer id;

    API_CLASS(int _id) {
      id = _id;
    }

    public int getId() {
      return this.id;
    }
  }

  public enum API_INDEX {
    ENABLE_CONTROL(0),
    DISABLE_CONTROL(1),
    SET_SETPOINT(2),
    P_CONSTANT(3),
    I_CONSTANT(4),
    D_CONSTANT(5),
    SET_REFERENCE(6),
    TRUSTED_ENABLED(7),
    TRUSTED_SET_NO_ACK(8),
    TRUSTED_SET_POINT_NO_ACK(10),
    SET_POINT_NO_ACK(11);

    private Integer id;

    API_INDEX(int _id) {
      id = _id;
    }

    public int getId() {
      return this.id;
    }
  }

  public enum BROADCAST_MESSAGE {
    DISABLE(0),
    SYSTEM_HALT(1),
    SYSTEM_RESET(2),
    DEVICE_ASSIGN(3),
    DEVICE_QUERY(4),
    HEARTBEAT(5),
    SYNC(6),
    UPDATE(7),
    FIRMWARE_VERSION(8),
    ENUMERATE(9),
    SYSTEM_RESUME(10);

    private Integer id;

    BROADCAST_MESSAGE(int _id) {
      id = _id;
    }

    public int getId() {
      return this.id;
    }
  }
}
