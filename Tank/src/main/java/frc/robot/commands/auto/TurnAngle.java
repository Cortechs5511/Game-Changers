package frc.robot.commands.auto;

import frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnAngle extends CommandBase {
	private final Drive m_drive;

	private double startingAngle;
	private double input;
	private final double setpoint;
	private final double threshold;

	public TurnAngle(double set, double error, Drive drive) {
		threshold = error;
		setpoint = set;
		m_drive = drive;
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		startingAngle = m_drive.getHeading();
	}

	@Override
	public void execute() {
		input = m_drive.getHeading() - startingAngle;

		double val = m_drive.anglePID.calculate(input, setpoint);

		if (Math.abs(val) > 0.35) {
			val = (0.35 * val / Math.abs(val));
		} else if (Math.abs(val) < 0.05) {
			val = 0;
		}

		m_drive.setLeft(val);
		m_drive.setRight(-val);
	}

	@Override
	public void end(boolean interrupted) {
		m_drive.setLeft(0);
		m_drive.setRight(0);
	}

	@Override
	public boolean isFinished() {
		return (Math.abs(input) < threshold) && (m_drive.getLeftVelocity.get() < 30) && (m_drive.getRightVelocity.get() < 30);
	}
}
