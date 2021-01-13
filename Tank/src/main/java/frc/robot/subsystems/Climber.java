package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
	private CANSparkMax climb0 = new CANSparkMax(ClimberConstants.kClimb0Port, MotorType.kBrushless);
	private WPI_VictorSPX climb1 = new WPI_VictorSPX(ClimberConstants.kClimb1Port);
	//private CANEncoder climbEncoder = climb0.getEncoder();

	//public Supplier<Double> getClimbEncoder = () -> climbEncoder.getPosition();

	public Climber() {
		climb0.restoreFactoryDefaults();

		climb0.setIdleMode(IdleMode.kBrake);
		climb0.setSecondaryCurrentLimit(250);
		climb0.setSmartCurrentLimit(60, 60, 200000);
		climb0.setOpenLoopRampRate(0.5);

		//climbEncoder.setPositionConversionFactor(42);
	}

	public void climbUp(double climberInput) {
		climb1.set(climberInput);
	}

	public void climbDown(double climberInput) {
		climb0.set(climberInput);
	}

	@Override
	public void periodic() {
	}
}