package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import frc.robot.commands.LightOn;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Shoot extends SequentialCommandGroup {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  public Shoot(int feedThreshold, Shooter shooter, Feeder feeder, Limelight limelight, Intake intake) {
    addCommands(
        new LightOn(limelight),
        new WaitCommand(0.5), 
        new Accel(shooter, limelight), 
        new Cruise(feedThreshold, shooter, feeder, limelight, intake));
  }
}