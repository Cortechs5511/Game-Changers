package frc.robot.commands.drive;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LimelightAlign extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private Drive m_drive;
	private Limelight m_limelight;
	private double val;
	private double input;
	private double threshold = 0.5;

	public LimelightAlign(Drive drive, Limelight limelight) {
		m_drive = drive;
		m_limelight = limelight;
		addRequirements(drive);
		addRequirements(limelight);
	}

	@Override
	public void initialize() {
		//SmartDashboard.putBoolean("Limelight Align Active", true);
		m_drive.anglePID.reset();
	}

	@Override
	public void execute() {
		input = m_limelight.getX();
		threshold = SmartDashboard.getNumber("Threshold", 0.5);
		//SmartDashboard.putNumber("Limelight Received X", input);
		double setpoint = 0.0;

		val = m_drive.anglePID.calculate(input, setpoint);
		//SmartDashboard.putNumber("Raw Val", val);

		if (Math.abs(val) > 0.35) {
			val = (0.35 * val / Math.abs(val));
		} else if (Math.abs(val) < 0.05) {
			val = 0;
		}
		//SmartDashboard.putNumber("Post Deadband Val", val);
		m_drive.setLeft(val);
		m_drive.setRight(-val);
	}

	@Override
	public void end(boolean interrupted) {
		m_drive.setLeft(0);
		m_drive.setRight(0);
		//SmartDashboard.putBoolean("Limelight Align Active", false);
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
