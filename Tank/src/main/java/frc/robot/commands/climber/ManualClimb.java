package frc.robot.commands.climber;

import frc.robot.OI;
import frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualClimb extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

	private Climber m_climber;
	private OI m_oi = OI.getInstance();
	
	public ManualClimb(Climber climber) {
		m_climber = climber;
		addRequirements(climber);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		if (m_oi.getClimberUp.get()) {
			m_climber.climbUp(0.8);
		} else {
			m_climber.climbUp(0);
		}

		if (m_oi.getClimberDown.get()) {
			m_climber.climbDown(0.8);
		} else {
			m_climber.climbDown(0);
		}
	}

	@Override
	public void end(boolean interrupted) {
		m_climber.climbUp(0);
		m_climber.climbDown(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
