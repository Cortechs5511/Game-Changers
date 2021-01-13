package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class OI {
	private static OI oi;

	private Joystick leftStick = new Joystick(0);
	private Joystick rightStick = new Joystick(1);
	private XboxController controller = new XboxController(2);

	public Supplier<Double> getLeftY = () -> leftStick.getY();
	public Supplier<Double> getRightY = () -> rightStick.getY();

	public Supplier<Boolean> getWristUp = () -> Math.abs(controller.getRawAxis(2)) > 0.5;
	public Supplier<Boolean> getWristDown = () -> Math.abs(controller.getRawAxis(3)) > 0.5;

	public Supplier<Boolean> getIntake = () -> controller.getRawButton(5) || controller.getRawButton(6);

	public Supplier<Boolean> getIntakeBackFeed = () -> 
			(Math.abs(controller.getRawAxis(0)) > 0.7) || 
			(Math.abs(controller.getRawAxis(1)) > 0.7);
			
	public Supplier<Boolean> getBeltBackFeed = () -> 
			(Math.abs(controller.getRawAxis(4)) > 0.7) || 
			(Math.abs(controller.getRawAxis(5)) > 0.7);

	public Supplier<Boolean> getClimberUp = () -> (controller.getPOV()) == 0;
	public Supplier<Boolean> getClimberDown = () -> (controller.getPOV()) == 180;

	private OI() {
	}

	public void setLeftRumble(double input) {
		controller.setRumble(RumbleType.kLeftRumble, input);
	}

	public void setRightRumble(double input) {
		controller.setRumble(RumbleType.kRightRumble, input);
	}

	public static OI getInstance() {
		if (oi == null) {
			oi = new OI();
		}
		return oi;
	}
}