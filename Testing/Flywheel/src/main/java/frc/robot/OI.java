package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import java.util.function.Supplier;

public class OI {
    private static OI oi;

    private final Joystick leftStick = new Joystick(Constants.OIConstants.kLeftStickPort);
    private final Joystick rightStick = new Joystick(Constants.OIConstants.kRightStickPort);
    private final XboxController controller = new XboxController(Constants.OIConstants.kControllerPort);

    public Supplier<Double> getLeftY = leftStick::getY;
    public Supplier<Double> getRightY = rightStick::getY;

    public Supplier<Boolean> getWristUp = () -> Math.abs(controller.getRawAxis(2)) > 0.5;
    public Supplier<Boolean> getWristDown = () -> Math.abs(controller.getRawAxis(3)) > 0.5;

    public Supplier<Boolean> getIntake = () -> controller.getRawButton(5) || controller.getRawButton(6) || leftStick.getTrigger();

    public Supplier<Boolean> getIntakeBackFeed = () ->
            (Math.abs(controller.getRawAxis(0)) > 0.7) ||
                    (Math.abs(controller.getRawAxis(1)) > 0.7);

    public Supplier<Boolean> getBeltBackFeed = () ->
            (Math.abs(controller.getRawAxis(4)) > 0.7) ||
                    (Math.abs(controller.getRawAxis(5)) > 0.7);

    private OI() {
    }

    public static OI getInstance() {
        if (oi == null) {
            oi = new OI();
        }
        return oi;
    }

    public void setLeftRumble(double input) {
        controller.setRumble(RumbleType.kLeftRumble, input);
    }

    public void setRightRumble(double input) {
        controller.setRumble(RumbleType.kRightRumble, input);
    }
}