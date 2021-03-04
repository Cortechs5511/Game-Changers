package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class Cruise extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Shooter m_shooter;
    private final Feeder m_feeder;
    private final Intake m_intake;
    private final OI m_oi = OI.getInstance();
    private final int threshold;
    private double target;
    private int count = 0;
    private int feedCount = 0;

    public Cruise(int feedThreshold, Shooter shooter, Feeder feeder, Limelight limelight, Intake intake) {
        threshold = feedThreshold;
        m_shooter = shooter;
        m_feeder = feeder;
        m_intake = intake;

        addRequirements(shooter);
        addRequirements(feeder);
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        target = m_shooter.getTarget.get();
        m_intake.setIntake(0.4);
    }

    @Override
    public void execute() {
        if (!m_feeder.getTopSensor.get()) { // if there is a ball in the top, and stabilized
            count = 0; // count is 0
        } else if (feedCount > threshold) { // else (there is no ball, or the sensor is dead), timer starts counting
            count++;
        }

        if ((Math.abs(m_shooter.getSpeed.get() - target)) < 50) {
            feedCount++;
        } else {
            feedCount = 0;
        }

        if (feedCount > threshold) {
            m_feeder.setFeederSpeed(0.7); // .45
            m_feeder.setFeeder2Speed(0.8);
            m_feeder.setFeeder3Speed(0.35);
        } else {
            m_feeder.setFeederSpeed(0);
            m_feeder.setFeeder2Speed(0);
            m_feeder.setFeeder3Speed(0);
        }

        if ((count > 25) && (count < 45)) {
            m_oi.setLeftRumble(1);
            m_oi.setRightRumble(1);
        } else {
            m_oi.setLeftRumble(0);
            m_oi.setRightRumble(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        count = 0;

        m_oi.setLeftRumble(0);
        m_oi.setRightRumble(0);

        m_shooter.setRampRate(1.5);
        m_shooter.setPIDReference(0);
        m_shooter.setSpeed(0);
        // m_shooter.setSpeed(0.7);

        m_feeder.setFeederSpeed(0);
        m_feeder.setFeeder2Speed(0);
        m_feeder.setFeeder3Speed(0);
    }

    @Override
    
    public boolean isFinished() {
        return ((count > 100) || (m_shooter.getSpeed.get() < 1000)); // about 2
        // seconds
        // of pause
        // or if
        // underspeed
        // return ((count > 100) || (m_shooter.getSpeed.get() < 1000));
    }
}