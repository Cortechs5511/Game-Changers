package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class StopShooter extends CommandBase {
    private final Shooter m_shooter;
    private final Feeder m_feeder;
    private final Drive m_drive;
    private final Limelight m_limelight;
    private final OI m_oi = OI.getInstance();

    private int count = 0;

    public StopShooter(Shooter shooter, Limelight limelight, Feeder feeder, Drive drive) {
        m_shooter = shooter;
        m_limelight = limelight;
        m_feeder = feeder;
        m_drive = drive;
        addRequirements(shooter);
        addRequirements(limelight);
        addRequirements(feeder);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_feeder.setFeederSpeed(0);
        m_feeder.setFeeder2Speed(0);
        m_feeder.setFeeder3Speed(0);

        m_shooter.setRampRate(0.75);

        m_drive.set(0, 0);

        m_oi.setLeftRumble(1); // rumble to 1 to try to prevent stuck motor input
        m_oi.setRightRumble(1);

        m_limelight.setLightStatus(1);
    }

    @Override
    public void execute() {
        count++;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.setPIDReference(0);
        m_shooter.setOutput(0);

        m_oi.setLeftRumble(0);
        m_oi.setRightRumble(0);
    }

    @Override
    public boolean isFinished() {
        return (count > 10); 
    }
}