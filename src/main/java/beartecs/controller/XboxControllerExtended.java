package beartecs.controller;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;

public class XboxControllerExtended extends XboxController {

  private final DPadButton[] dpadButtons;

  public XboxControllerExtended(int port) {
    super(port);
    dpadButtons = new DPadButton[DPadButton.Direction.values().length];
    for (DPadButton.Direction dir : DPadButton.Direction.values()) {
      dpadButtons[dir.ordinal()] = new DPadButton(this, dir);
    }
  }

  public void setLeftVibration(double value) {
    this.setRumble(GenericHID.RumbleType.kLeftRumble, value);
  }

  public void setRightVibration(double value) {
    this.setRumble(GenericHID.RumbleType.kRightRumble, value);
  }

  public DPadButton getDPadButton(DPadButton.Direction direction) {
    return dpadButtons[direction.ordinal()];
  }

  public boolean getDPadUp() {
    return getDPadButton(DPadButton.Direction.UP).get();
  }

  public boolean getDPadLeft() {
    return getDPadButton(DPadButton.Direction.LEFT).get();
  }

  public boolean getDPadRight() {
    return getDPadButton(DPadButton.Direction.RIGHT).get();
  }

  public boolean getDPadDown() {
    return getDPadButton(DPadButton.Direction.DOWN).get();
  }
}
