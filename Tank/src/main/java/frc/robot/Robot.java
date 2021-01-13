package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	Command m_autonomousCommand;

	private RobotContainer m_robotContainer;

	// ROBOT INIT
	@Override
	public void robotInit() {
		m_robotContainer = new RobotContainer();
	}

	// ROBOT PERIODIC
	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	// AUTO INIT
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();
		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
		
	}

	// AUTO PERIODIC
	@Override
	public void autonomousPeriodic() {
	
	}

	// TELEOP INIT
	@Override
	public void teleopInit() {
		m_robotContainer.teleopInit(this);
	}

	// TELEOP PERIODIC
	@Override
	public void teleopPeriodic() {
	}
}