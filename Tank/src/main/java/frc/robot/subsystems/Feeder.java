package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

//import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase {
	private WPI_VictorSPX feeder0 = new WPI_VictorSPX(FeederConstants.kFeeder0Port); // feeder 0 and 1 are tower
	private WPI_VictorSPX feeder1 = new WPI_VictorSPX(FeederConstants.kFeeder1Port);
	private WPI_VictorSPX feeder2 = new WPI_VictorSPX(FeederConstants.kFeeder2Port); // feeder 2, 3 are feeder wheels
	private WPI_VictorSPX feeder3 = new WPI_VictorSPX(FeederConstants.kFeeder3Port);

	private DigitalInput bottomSensor = new DigitalInput(FeederConstants.kBottomSensorPort);
	private DigitalInput topSensor = new DigitalInput(FeederConstants.kTopSensorPort);
	private DigitalInput greenSensor = new DigitalInput(FeederConstants.kGreenSensorPort);
	private DigitalInput blackSensor = new DigitalInput(FeederConstants.kBlackSensorPort);

	private Encoder intakeEncoder = new Encoder(4, 5); // wrist -- inop
	private Encoder feedEncoder = new Encoder(6, 7); // tower -- inop

	public Supplier<Boolean> getBottomSensor = () -> bottomSensor.get();
	public Supplier<Boolean> getTopSensor = () -> topSensor.get();
	public Supplier<Boolean> getGreenSensor = () -> greenSensor.get();
	public Supplier<Boolean> getBlackSensor = () -> blackSensor.get();

	// public Supplier<Integer> getSumCounter = () -> sumCounter;

	public Supplier<Integer> getIntakeEncoder = () -> intakeEncoder.get();
	public Supplier<Integer> getFeedEncoder = () -> feedEncoder.get();

	public Feeder() {
		feeder0.configFactoryDefault();
		feeder1.configFactoryDefault();
		feeder2.configFactoryDefault();
		feeder2.configFactoryDefault();

		feeder0.setNeutralMode(NeutralMode.Brake);
		feeder1.setNeutralMode(NeutralMode.Brake);
		feeder2.setNeutralMode(NeutralMode.Brake);
		feeder2.setNeutralMode(NeutralMode.Brake);

		feeder0.setInverted(false);
		feeder1.setInverted(true);
		feeder2.setInverted(true);
		feeder3.setInverted(true);
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("Top Sensor", topSensor.get());
		SmartDashboard.putBoolean("Bottom Sensor", bottomSensor.get());
		SmartDashboard.putBoolean("Green Sensor", greenSensor.get());
		SmartDashboard.putBoolean("Black Sensor", blackSensor.get());
		// sumCounter = bottomCounter.get();

		//SmartDashboard.putNumber("Feeder Wheels Encoder", intakeEncoder.get()); // intakeEncoder in feeder subsystem =
																				// black wheels
		//SmartDashboard.putNumber("Tower Encoder", feedEncoder.get());

		// SmartDashboard.putNumber("Balls Shot", topCounter.get());
		// SmartDashboard.putNumber("Ball Counter Period", topCounter.getPeriod());
	}

	public void setFeederSpeed(double input) {
		feeder0.set(input);
		feeder1.set(input);
	}

	public void setFeeder2Speed(double input) {
		feeder2.set(input);
	}

	public void setFeeder3Speed(double input) {
		feeder3.set(input);
	}

	public void resetFeedEncoder() {
		feedEncoder.reset();
	}
}