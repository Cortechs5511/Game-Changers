package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.auto.DriveTimed;
import frc.robot.commands.auto.TurnAngle;
import frc.robot.commands.drive.Brake;
import frc.robot.commands.drive.Coast;
import frc.robot.commands.shooter.Shoot;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class TrenchSimple extends SequentialCommandGroup {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	public TrenchSimple(double startDelay, Shooter shooter, Feeder feeder, Limelight limelight, Drive drive, Intake intake) {
		addCommands(
			new Brake(drive), // sets all motors to brake mode
			new WaitCommand(startDelay), // optional start delay
			new Shoot(-1, shooter, feeder, limelight, intake), // fast shoot
			new TurnAngle(-20, 0.5, drive), // turn 20 degrees by gyro w/ 0.5 degree tolerance
			new DriveTimed(drive, 1, 0.4), // drive 1 second back at 0.4 power
			new WaitCommand(1), // wait 1 sec to stop (brake mode)
			new Coast(drive)); // sets all motors to coast mode
	}
}