package frc.robot.commands.shooter;

import frc.robot.OI;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AccelAlign extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private Drive m_drive;
	private Limelight m_limelight;
	private Shooter m_shooter;
	private OI m_oi = OI.getInstance();

	private double val, input, currentSpeed, calculatedSpeed;
	private double threshold;
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
		m_shooter.setRampRate(1.5);
		calculatedSpeed = m_limelight.calculateRPM();
		m_shooter.setPIDReference(calculatedSpeed);

		m_drive.anglePID.reset();
	}

	@Override
	public void execute() {
		input = m_limelight.getX();
		// SmartDashboard.putNumber("Limelight Received X", input);
		double setpoint = 0.0;

		val = m_drive.anglePID.calculate(input, setpoint);
		// SmartDashboard.putNumber("Raw Val", val);

		if (Math.abs(val) > 0.35) {
			val = (0.35 * val / Math.abs(val));
		} else if (Math.abs(val) < 0.05) {
			val = 0;
		}

		// SmartDashboard.putNumber("Post Deadband Val", val);
		m_drive.setLeft(val);
		m_drive.setRight(-val);

		currentSpeed = m_shooter.getSpeed.get();

		if (Math.abs(calculatedSpeed - currentSpeed) < 200) {
			count++;
		} else {
			count = 0;
		}
		if (count != 0) {
			m_oi.setLeftRumble(0.3);
			m_oi.setRightRumble(0.3);
		}

		// System.out.println(currentSpeed);
	}

	@Override
	public void end(boolean interrupted) {
		calculatedSpeed = m_limelight.calculateRPM();
		m_shooter.setPIDReference(calculatedSpeed);
		m_limelight.setLightStatus(1);

		m_drive.setLeft(0);
		m_drive.setRight(0);
		//SmartDashboard.putBoolean("Limelight Align Active", false);

		m_shooter.setRampRate(0.15);
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
