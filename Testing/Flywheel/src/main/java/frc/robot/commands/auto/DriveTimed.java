package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveTimed extends CommandBase {
	private Timer timer = new Timer();
	private Drive m_drive;
	private double stop, speed;

	public DriveTimed(Drive drive, double time, double power) {
		stop = time;
		speed = power;
		m_drive = drive;
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		timer.reset();
		timer.start();

		m_drive.setLeft(speed);
		m_drive.setRight(speed);
	}

	@Override
	public void execute() {
	}

	@Override
	public void end(boolean interrupted) {
		m_drive.setLeft(0);
		m_drive.setRight(0);
	}

	@Override
	public boolean isFinished() {
		return (timer.get() > stop);
	}
}
