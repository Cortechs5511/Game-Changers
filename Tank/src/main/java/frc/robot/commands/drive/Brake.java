package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drive;

public class Brake extends InstantCommand {
	private Drive m_drive;

	public Brake(Drive drive) {
		m_drive = drive;
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		m_drive.setBrake();
	}
}
