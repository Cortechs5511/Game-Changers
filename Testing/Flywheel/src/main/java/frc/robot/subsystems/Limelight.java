package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

public class Limelight extends SubsystemBase {
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final NetworkTableEntry tx = table.getEntry("tx");
    private final NetworkTableEntry ty = table.getEntry("ty");
    private final NetworkTableEntry ta = table.getEntry("ta");
    private final NetworkTableEntry tv = table.getEntry("tv");
    private final NetworkTableEntry ledMode = table.getEntry("ledMode");
    private double x;
    private double y;
    private double v;
    public Supplier<Boolean> invalidTarget = () -> !(v == 1);
    private double area;
    private double RPMAdjustment;
    private double flatRPM;
    private boolean RPMMode = false;

    public Limelight() {
        ledMode.setNumber(1); // sets lights off
        SmartDashboard.putNumber("Shooter/RPM Setpoint", 0);
        SmartDashboard.putNumber("Shooter/RPM Adjustment", 0);
        SmartDashboard.putNumber("Shooter/Flat RPM", 4050);
        SmartDashboard.putBoolean("Shooter/RPM Flat Control", RPMMode);
        SmartDashboard.putBoolean("Limelight/Limelight Lights", false);
    }

    @Override
    public void periodic() {
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        v = tv.getDouble(0.0);
        area = ta.getDouble(0.0);

        SmartDashboard.putNumber("Limelight/Limelight X", x);
        SmartDashboard.putBoolean("Limelight/Limelight Valid Target", v == 0);

        RPMAdjustment = SmartDashboard.getNumber("Shooter/RPM Adjustment", 0);
        flatRPM = SmartDashboard.getNumber("Shooter/Flat RPM", 4050);
    }

    public double calculateRPM() {
        double distance = ((63.65) / Math.tan(Math.toRadians(y + 16.94))) * (((-Math.abs(y)) / 300) + 1);
        RPMMode = SmartDashboard.getBoolean("Shooter/RPM Flat Control", false);

        // 0.00913x^2 -2.69x + 3647
        double rpm = (0.00913 * Math.pow(distance, 2)) - (2.69 * distance) + 3647 + RPMAdjustment;
        SmartDashboard.putNumber("Shooter/RPM Setpoint", rpm);

        if (RPMMode) {
            return flatRPM;
        } else if (v != 0) {
            return rpm;
        } else {
            return 0;
        }
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getArea() {
        return area;
    }

    public double getLightStatus() {
        return ledMode.getDouble(1);
    }

    public void setLightStatus(double input) {
        ledMode.setNumber(input); // 3 = on, 1 = off
        SmartDashboard.putBoolean("Limelight/Limelight Lights", input != 1);
    }
}