package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FeederConstants;

public class Intake extends SubsystemBase {
	private TalonSRX wrist = new TalonSRX(FeederConstants.kWristPort);
	private WPI_VictorSPX intake = new WPI_VictorSPX(FeederConstants.kIntakePort);

	public Intake() {
		wrist.configFactoryDefault();
		intake.configFactoryDefault();

		wrist.setNeutralMode(NeutralMode.Brake);
		intake.setNeutralMode(NeutralMode.Brake);

		intake.setInverted(true);
		intake.configOpenloopRamp(0.1);
	}

	@Override
	public void periodic() {
	}

	public void setWrist(double power) {
		wrist.set(ControlMode.PercentOutput, power);
	}

	public void setIntake(double power) {
		intake.set(power);
	}
}