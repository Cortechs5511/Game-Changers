package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drive;
import frc.robot.OI;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetIntakePower extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final Intake m_intake;
	private final Drive m_drive;
	private final OI m_oi = OI.getInstance();

	public SetIntakePower(Intake intake, Drive drive) {
		m_intake = intake;
		m_drive = drive;

		addRequirements(intake);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		if (m_oi.getWristDown.get()) {
			m_intake.setWrist(0.3);
		} else if (m_oi.getWristUp.get()) {
			m_intake.setWrist(-0.48);
		} else {
			m_intake.setWrist(0);
		}

		if (m_oi.getIntake.get()) {
			m_intake.setIntake(0.75);
		} else if (m_oi.getIntakeBackFeed.get()) {
			m_intake.setIntake(-0.5);
		} else {
			// stop intake
			m_intake.setIntake(0);
		}

	}

	@Override
	public void end(boolean interrupted) {
		m_intake.setIntake(0);
		m_intake.setWrist(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}