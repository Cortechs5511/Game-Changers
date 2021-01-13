package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drive;

public class StopDrive extends InstantCommand {
	private Drive m_drive;

	public StopDrive(Drive drive) {
		m_drive = drive;
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		m_drive.setBrake();
		m_drive.setLeft(0);
		m_drive.setRight(0);
	}
}
