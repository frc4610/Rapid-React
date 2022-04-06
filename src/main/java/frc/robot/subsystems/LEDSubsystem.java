package frc.robot.subsystems;

import beartecs.Constants.*;
import beartecs.LED.*;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;

//https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_ledStrip;
  private final AddressableLEDBuffer m_ledBuffer;

  public static final SolidColorPattern m_bluePattern = new SolidColorPattern(Color.kBlue);
  public static final SolidColorPattern m_redPattern = new SolidColorPattern(Color.kRed);
  public static final SolidColorPattern m_greenPattern = new SolidColorPattern(Color.kGreen);
  public static final SolidColorPattern m_yellowPattern = new SolidColorPattern(Color.kYellow);

  public static final BlinkingPattern m_blinkingYellow = new BlinkingPattern(Color.kYellow, 0.4);
  public static final RainbowPattern m_rainbowPattern = new RainbowPattern();
  public static final IntensityPattern m_blueIntensityPattern = new IntensityPattern(Color.kBlue, 0.5);
  public static final IntensityPattern m_redIntensityPattern = new IntensityPattern(Color.kRed, 0.5);
  public static final ScannerPattern m_scannerRedPattern = new ScannerPattern(Color.kRed, Color.kBlack, 30);
  public static final ScannerPattern m_scannerBluePattern = new ScannerPattern(Color.kBlue, Color.kBlack, 30);
  private static final Color[] chaseBlueColor = {
      Color.kBlue, Color.kBlue, Color.kBlue, Color.kBlue, Color.kBlue, Color.kBlue,
      Color.kBlue, Color.kBlue, Color.kBlue, Color.kBlue, Color.kBlue, Color.kBlue,
      Color.kBlue, Color.kBlue, Color.kBlue, Color.kBlue, Color.kBlue, Color.kBlue,
      Color.kBlue, Color.kBlue, Color.kBlue, Color.kBlue, Color.kBlue, Color.kBlue,
      Color.kWhite };
  public static final ChasePattern m_chaseBluePattern = new ChasePattern(chaseBlueColor, 1);
  private static final Color[] chaseRedColor = {
      Color.kRed, Color.kRed, Color.kRed, Color.kRed, Color.kRed, Color.kRed,
      Color.kRed, Color.kRed, Color.kRed, Color.kRed, Color.kRed, Color.kRed,
      Color.kRed, Color.kRed, Color.kRed, Color.kRed, Color.kRed, Color.kRed,
      Color.kRed, Color.kRed, Color.kRed, Color.kRed, Color.kRed, Color.kRed,
      Color.kWhite };
  public static final ChasePattern m_chaseRedPattern = new ChasePattern(chaseRedColor, 1);
  private static final Color[] greenAlternatingColor = { Color.kGreen, Color.kLime };
  public static final AlternatingColorPattern m_greenAlternating = new AlternatingColorPattern(greenAlternatingColor);
  public static TimerPattern m_timerPattern = new TimerPattern(Color.kWhite, 15.0);

  public static final int LED_STRIP_COUNT = 58;

  public LEDSubsystem() {
    m_ledStrip = new AddressableLED(Ids.PWM_LED_STRIP);
    m_ledBuffer = new AddressableLEDBuffer(LED_STRIP_COUNT);
    m_ledStrip.setLength(LED_STRIP_COUNT);
    m_ledStrip.setData(m_ledBuffer);
    m_ledStrip.start();

  }

  public void setPattern(AddressableLEDPattern pattern) {
    pattern.setLEDs(m_ledBuffer);
  }

  public void setAllianceColors() {
    switch (DriverStation.getAlliance()) {
      case Blue:
        setPattern(m_chaseBluePattern);
        break;
      case Red:
        setPattern(m_chaseRedPattern);
        break;
      default:
        setPattern(m_rainbowPattern);
        break;
    }
  }

  public void setAllianceAutonomousColors() {
    switch (DriverStation.getAlliance()) {
      case Blue:
        setPattern(m_scannerBluePattern);
        break;
      case Red:
        setPattern(m_scannerRedPattern);
        break;
      default:
        setPattern(m_rainbowPattern);
        break;
    }
  }

  @Override
  public void periodic() {
    m_ledStrip.setData(m_ledBuffer);
  }
}