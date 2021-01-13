package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
  private double x, y, v, area, distance, RPMAdjustment, flatRPM;
  private boolean RPMMode = false;
  
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry ta = table.getEntry("ta");
  private NetworkTableEntry tv = table.getEntry("tv");
  private NetworkTableEntry ledMode = table.getEntry("ledMode");

  public Supplier<Boolean> invalidTarget = () -> !(v == 1);

  public Limelight() {
	ledMode.setNumber(1); // sets lights off
	SmartDashboard.putNumber("RPM Setpoint", 0);
	SmartDashboard.putNumber("RPM Adjustment", 0);
	SmartDashboard.putNumber("Flat RPM", 4050);
	SmartDashboard.putBoolean("RPM Flat Control", RPMMode);
	SmartDashboard.putBoolean("Limelight Lights", false);
  }

  @Override
  public void periodic() {
	x = tx.getDouble(0.0);
	y = ty.getDouble(0.0);
	v = tv.getDouble(0.0);
	area = ta.getDouble(0.0);

	SmartDashboard.putNumber("Limelight X", x);
	SmartDashboard.putBoolean("Limelight Valid Target", v == 0);

	RPMAdjustment = SmartDashboard.getNumber("RPM Adjustment", 0);
	flatRPM = SmartDashboard.getNumber("Flat RPM", 4050);

	//SmartDashboard.putNumber("Y", y);
	//SmartDashboard.putNumber("Area", area);
	
	//distance = ((63.65) / Math.tan(Math.toRadians(y + 16.94))) * (((-Math.abs(y)) / 300) + 1); // in periodic for testing
	//SmartDashboard.putNumber("Calculated Distance", distance);
  }

  public double calculateRPM() {
	distance = ((63.65) / Math.tan(Math.toRadians(y + 16.94))) * (((-Math.abs(y)) / 300) + 1);
	RPMMode = SmartDashboard.getBoolean("RPM Flat Control", false);
	//SmartDashboard.putNumber("Calculated Distance", distance);
	
	// 0.00913x^2 -2.69x + 3647
	double rpm = (0.00913 * Math.pow(distance, 2)) - (2.69 * distance) + 3647 + RPMAdjustment;
	SmartDashboard.putNumber("RPM Setpoint", rpm);

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
	if (input == 1) {
	  SmartDashboard.putBoolean("Limelight Lights", false);
	} else {
	  SmartDashboard.putBoolean("Limelight Lights", true);
	}
  }
}