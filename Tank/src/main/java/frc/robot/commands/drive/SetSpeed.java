package frc.robot.commands.drive;

import frc.robot.subsystems.Drive;
import frc.robot.OI;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetSpeed extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private OI m_oi = OI.getInstance();
	private double leftInput, rightInput, leftOutput, rightOutput;
	private Drive m_drive;

	public SetSpeed(Drive drive) {
		m_drive = drive;
		addRequirements(drive);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		leftInput = m_oi.getLeftY.get();
		rightInput = m_oi.getRightY.get();

		if (Math.abs(leftInput) > 0.1) {
			leftOutput = leftInput * m_drive.getMaxOutput.get();
		} else {
			leftOutput = 0;
		}
		if (Math.abs(rightInput) > 0.1) {
			rightOutput = rightInput * m_drive.getMaxOutput.get();
		} else {
			rightOutput = 0;
		}

		if (m_drive.invert == false) {
			m_drive.setLeft(leftOutput);
			m_drive.setRight(rightOutput);
		} else {
			m_drive.setLeft(-rightOutput);
			m_drive.setRight(-leftOutput);
		}
	}

	@Override
	public void end(boolean interrupted) {
		m_drive.setLeft(0);
		m_drive.setRight(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
