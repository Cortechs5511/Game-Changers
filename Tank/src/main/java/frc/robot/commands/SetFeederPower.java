package frc.robot.commands;

import frc.robot.subsystems.Feeder;
import frc.robot.OI;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetFeederPower extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private Feeder m_feeder;
	private OI m_oi = OI.getInstance();
	private double polySpeed, blackSpeed, greenSpeed;

	public SetFeederPower(Feeder feeder) {
		m_feeder = feeder;
		addRequirements(feeder);
	}

	@Override
	public void initialize() {
		m_feeder.resetFeedEncoder();
	}

	@Override
	public void execute() {
		boolean bottom = !m_feeder.getBottomSensor.get();
		boolean top = !m_feeder.getTopSensor.get();
		boolean green = !m_feeder.getGreenSensor.get();
		boolean black = !m_feeder.getBlackSensor.get();

		boolean intaking = m_oi.getIntake.get();

		if (intaking) {
			blackSpeed = 0.7;
			greenSpeed = 0.6;
		} else {
			blackSpeed = 0;
			greenSpeed = 0;
		}

		if (top && bottom && black && green) { // 1111
			polySpeed = 0; // poly off
			blackSpeed = 0; // black off
			greenSpeed = 0; // green off
		} else if (top && bottom && black && green == false) { // 1110
			polySpeed = 0; // poly off
			blackSpeed = 0; // black off, green on intake
		} else if (top && bottom && black == false && green) { // 110*
			polySpeed = 0; // poly off, black and green on intake
		} else if (top && bottom && black == false && green == false) { // 1100
			polySpeed = 0; // poly off, black and green on intake
		} else if (top && bottom == false && black && green) { // 1011
			polySpeed = -0.4; // poly back
			blackSpeed = 0; // black off
			greenSpeed = 0; // green off
		} else if (top && bottom == false && black == false && green) { // 1001
			polySpeed = -0.4; // poly back, black and green on intake
		} else if (top && bottom == false && black && green == false) { // 1010
			polySpeed = -0.4; // poly back
			blackSpeed = 0; // black off, green on intake
		} else if (top && bottom == false && black == false && green == false) { // 1000
			polySpeed = -0.4; // poly back, black and green on intake
		} else if (top == false && bottom) { // 01**
			polySpeed = 0.4; // poly on, black and green on intake
		} else if (top == false && bottom == false) { // 00**
			polySpeed = 0; // poly off, black and green on intake
		}

		if (m_oi.getIntakeBackFeed.get()) {
			blackSpeed = -0.6;
			greenSpeed = -0.6;
		}
		if (m_oi.getBeltBackFeed.get()) {
			polySpeed = -0.4;
		}
		m_feeder.setFeederSpeed(polySpeed); // poly
		m_feeder.setFeeder2Speed(blackSpeed); // black
		m_feeder.setFeeder3Speed(greenSpeed); // green
	}

	@Override
	public void end(boolean interrupted) {
		m_feeder.setFeederSpeed(0);
		m_feeder.setFeeder2Speed(0);
		m_feeder.setFeeder3Speed(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}