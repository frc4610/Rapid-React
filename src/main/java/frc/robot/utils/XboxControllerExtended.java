package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utils.Controller.DPadButton;

public class XboxControllerExtended extends XboxController {

  private final DPadButton[] dpadButtons;
  public XboxControllerExtended(int port) {
    super(port);
    dpadButtons = new DPadButton[DPadButton.Direction.values().length];

		for (DPadButton.Direction dir : DPadButton.Direction.values()) {
			dpadButtons[dir.ordinal()] = new DPadButton(this, dir);
		}
  }
  
  public DPadButton getDPadButton(DPadButton.Direction direction) {
		return dpadButtons[direction.ordinal()];
	}

  public boolean getDPadUp() {
		return getDPadButton(DPadButton.Direction.UP).getAsBoolean();
	}
  public boolean getDPadLeft() {
		return getDPadButton(DPadButton.Direction.LEFT).getAsBoolean();
	}
  public boolean getDPadRight() {
		return getDPadButton(DPadButton.Direction.LEFT).getAsBoolean();
	}
}
