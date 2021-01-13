package frc.robot.commands.auto;

import frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnAngle extends CommandBase {
	private Drive m_drive;

	private double startingAngle, val, input, setpoint;
	private double threshold;

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

		val = m_drive.anglePID.calculate(input, setpoint);

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
		if ((Math.abs(input) < threshold) && (m_drive.getLeftVelocity.get() < 30) && (m_drive.getRightVelocity.get() < 30)) {
			return true;
		} else {
			return false;
		}
	}
}
