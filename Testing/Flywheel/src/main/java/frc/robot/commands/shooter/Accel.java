package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class Accel extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Shooter m_shooter;
    private final OI m_oi = OI.getInstance();
    private final Limelight m_limelight;
    private double calculatedRPM;
    private int count = 0;

    public Accel(Shooter shooter, Limelight limelight) {
        m_shooter = shooter;
        m_limelight = limelight;
        addRequirements(shooter);
        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        m_shooter.setRampRate(1.5);
        calculatedRPM = m_limelight.calculateRPM();
        m_shooter.setPIDReference(calculatedRPM);
    }

    @Override
    public void execute() {
        double currentSpeed = m_shooter.getSpeed.get();

        if (Math.abs(calculatedRPM - currentSpeed) < 200) {
            count++;
        } else {
            count = 0;
        }

        if (count != 0) {
            m_oi.setLeftRumble(0.3);
            m_oi.setRightRumble(0.3);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_limelight.setLightStatus(1);
        m_shooter.setRampRate(0.05);
        count = 0;

        m_oi.setLeftRumble(0);
        m_oi.setRightRumble(0);
    }

    @Override
    public boolean isFinished() {
        return (count > 25);
    }
}