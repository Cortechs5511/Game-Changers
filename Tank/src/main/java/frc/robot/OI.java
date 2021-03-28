package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;

public class OI {
    private static OI oi;

    private final Joystick leftStick = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);

    public Supplier<Double> getLeftY = leftStick::getY;
    public Supplier<Double> getRightY = rightStick::getY;

    public static OI getInstance() {
        if (oi == null) {
            oi = new OI();
        }
        return oi;
    }
}