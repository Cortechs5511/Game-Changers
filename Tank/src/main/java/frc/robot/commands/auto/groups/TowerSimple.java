package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.auto.DriveTimed;
import frc.robot.commands.drive.Brake;
import frc.robot.commands.drive.Coast;
import frc.robot.commands.shooter.Shoot;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class TowerSimple extends SequentialCommandGroup {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	public TowerSimple(double startDelay, Shooter shooter, Feeder feeder, Limelight limelight, Drive drive, Intake intake) {
		addCommands(
			new Brake(drive), // sets all motors to brake mode
			new WaitCommand(startDelay), // optional start delay
			new Shoot(-1, shooter, feeder, limelight, intake), // fast shoot
			new DriveTimed(drive, 1, 0.4), // drive back for 1 second at 0.4 power
			new WaitCommand(1), // wait 1 second to let robot stop (brake mode)
			new Coast(drive)); // sets all motors to coast mode
	}
}