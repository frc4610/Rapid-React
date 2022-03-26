package globals.utils.controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;

public class DPadButton extends Button {

	public enum Direction {
		UP(0),
		UPRIGHT(45),
		RIGHT(90),
		DOWNRIGHT(135),
		DOWN(180),
		DOWNLEFT(225),
		LEFT(270),
		UPLEFT(315),
		CENTER(-1);

		private final int angle;

		Direction(int angle) {
			this.angle = angle;
		}

		public int getAngle() {
			return angle;
		}
	}

	private GenericHID hid;
	private Direction direction;
	private final int pov;

	public DPadButton(GenericHID hid, Direction direction, int pov) {
		this.hid = hid;
		this.direction = direction;
		this.pov = pov;
	}

	public DPadButton(GenericHID hid, Direction direction) {
		this(hid, direction, 0);
	}

	@Override
	public boolean get() {
		return hid.getPOV(pov) == direction.getAngle();
	}
}