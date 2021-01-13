package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
	private CANSparkMax shoot0 = new CANSparkMax(ShooterConstants.kShoot0Port, MotorType.kBrushless);
	private CANSparkMax shoot1 = new CANSparkMax(ShooterConstants.kShoot1Port, MotorType.kBrushless);

	private CANPIDController shootPID = shoot0.getPIDController();

	private double reference = 0f;

	private CANEncoder shootEnc = shoot0.getEncoder();

	public Supplier<Double> getSpeed = () -> shootEnc.getVelocity();
	public Supplier<Double> getTarget = () -> reference;

	public Shooter() {
		shoot0.restoreFactoryDefaults();
		shoot1.restoreFactoryDefaults();

		shoot0.setIdleMode(IdleMode.kCoast);
		shoot1.setIdleMode(IdleMode.kCoast);

		shoot0.enableVoltageCompensation(11);
		shoot1.enableVoltageCompensation(11);

		shoot0.setSecondaryCurrentLimit(250);
		shoot1.setSecondaryCurrentLimit(250);

		shoot0.setSmartCurrentLimit(200, 200, 200000);
		shoot1.setSmartCurrentLimit(200, 200, 200000);

		shoot0.setClosedLoopRampRate(1.5);
		shoot1.setClosedLoopRampRate(1.5);

		shoot0.setOpenLoopRampRate(1.5);
		shoot1.setOpenLoopRampRate(1.5);

		shoot0.setInverted(true);
		shoot1.setInverted(false);

		shootPID.setOutputRange(0, 1);

		shootPID.setP(0.000275);
		shootPID.setI(0);
		shootPID.setD(0.008);
		shootPID.setFF(0.000202);

		SmartDashboard.putNumber("Shooter P", 0.00004);
		SmartDashboard.putNumber("Shooter I", 0);
		SmartDashboard.putNumber("Shooter D", 0.00004);
		SmartDashboard.putNumber("Shooter FF", 0.00020);
		SmartDashboard.putNumber("RPM Adjustment", 0);
	}

	public void setSpeed(double input) {
		shoot0.set(input);
		shoot1.set(input);
	}

	public void setRampRate(double rate) {
		shoot0.setClosedLoopRampRate(rate);
		shoot1.setClosedLoopRampRate(rate);
	}

	public void setPIDReference(double ref) {
		reference = ref;
		shootPID.setReference(ref, ControlType.kVelocity);
	}

	@Override
	public void periodic() {
		shootPID.setP(SmartDashboard.getNumber("Shooter P", 0.00004));
		shootPID.setI(SmartDashboard.getNumber("Shooter I", 0));
		shootPID.setD(SmartDashboard.getNumber("Shooter D", 0.00004));
		shootPID.setFF(SmartDashboard.getNumber("Shooter FF", 0.00020));
		shoot1.set(shoot0.get());

		SmartDashboard.putNumber("Shooter rpm", shootEnc.getVelocity());
	}
}