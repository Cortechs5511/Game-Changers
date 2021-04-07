package frc.robot.commands.shooter;

import frc.robot.OI;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AccelAlign extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final Drive m_drive;
	private final Limelight m_limelight;
	private final Shooter m_shooter;
	private final OI m_oi = OI.getInstance();

	private double input;
	private double calculatedSpeed;
	private final double threshold;
	private int count = 0;

	public AccelAlign(double limeThreshold, Drive drive, Limelight limelight, Shooter shooter) {
		threshold = limeThreshold;
		m_drive = drive;
		m_limelight = limelight;
		m_shooter = shooter;
		addRequirements(drive);
		addRequirements(limelight);
		addRequirements(shooter);
	}

	@Override
	public void initialize() {
		m_shooter.setRampRate(0.75);
		calculatedSpeed = m_limelight.calculateRPM();
		m_shooter.setPIDReference(calculatedSpeed);

		m_drive.anglePID.reset();
	}

	@Override
	public void execute() {
		input = m_limelight.getX();
		double setpoint = 0.0;
		double val = m_drive.anglePID.calculate(input, setpoint);

		if (Math.abs(val) > 0.35) {
			val = (0.35 * val / Math.abs(val));
		} else if (Math.abs(val) < 0.01) {
			val = 0;
		}

		m_drive.set(val, -val);
		double currentSpeed = m_shooter.getSpeed.get();

		if (Math.abs(calculatedSpeed - currentSpeed) < 200) {
			count++;
		} else {
			count = 0;
		}
		if (count != 0) {
			m_oi.setLeftRumble(1.0);
			m_oi.setRightRumble(1.0);
		}
	}

	@Override
	public void end(boolean interrupted) {
		calculatedSpeed = m_limelight.calculateRPM();
		m_shooter.setPIDReference(calculatedSpeed);
		m_limelight.setLightStatus(1);

		m_drive.set(0, 0);
		m_shooter.setRampRate(0.01);
		count = 0;

		m_oi.setLeftRumble(0);
		m_oi.setRightRumble(0);
	}

	@Override
	public boolean isFinished() {
		return (((Math.abs(input) < threshold) && (m_drive.getLeftVelocity.get() < 30)
				&& (m_drive.getRightVelocity.get() < 30)) && (count > 25));
		//return false;
	}
}
