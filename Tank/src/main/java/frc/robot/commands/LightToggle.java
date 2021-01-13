package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Limelight;

public class LightToggle extends InstantCommand {
	private Limelight m_limelight;

	public LightToggle(Limelight limelight) {
		m_limelight = limelight;
		addRequirements(limelight);
	}

	@Override
	public void initialize() {
		double current = m_limelight.getLightStatus();
		if (current == 1) { // if off
			m_limelight.setLightStatus(3); // on
		} else {
			m_limelight.setLightStatus(1); // off
		}
	}
}
