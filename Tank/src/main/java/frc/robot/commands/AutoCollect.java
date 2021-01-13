package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class AutoCollect extends CommandBase {
	private Intake m_intake;
	private Feeder m_feeder;

	private double polySpeed, blackSpeed, greenSpeed;

	public AutoCollect(Intake intake, Feeder feeder) {
		m_intake = intake;
		m_feeder = feeder;
		
		addRequirements(intake);
		addRequirements(feeder);
	}

	@Override
	public void initialize() {
		m_intake.setIntake(0.75);
	}

	@Override
	public void execute() {
		// feeder
		boolean bottom = !m_feeder.getBottomSensor.get();
		boolean top = !m_feeder.getTopSensor.get();
		boolean green = !m_feeder.getGreenSensor.get();
		boolean black = !m_feeder.getBlackSensor.get();

		blackSpeed = 0.7;
		greenSpeed = 0.6;

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

		m_feeder.setFeederSpeed(polySpeed); // poly
		m_feeder.setFeeder2Speed(blackSpeed); // black
		m_feeder.setFeeder3Speed(greenSpeed); // green
	}

	@Override
	public void end(boolean interrupted) {
		m_intake.setIntake(0);
		m_feeder.setFeederSpeed(0); 
		m_feeder.setFeeder2Speed(0); 
		m_feeder.setFeeder3Speed(0); 
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
