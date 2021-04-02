package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Intake;

public class SetIntakePower extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake m_intake;
    private final OI m_oi = OI.getInstance();

    public SetIntakePower(Intake intake) {
        m_intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (m_oi.getWristDown.get()) {
            m_intake.setWrist(0.4);
        } else if (m_oi.getWristUp.get()) {
            m_intake.setWrist(-0.6);
        } else {
            m_intake.setWrist(0);
        }

        if (m_oi.getIntake.get()) {
            m_intake.setIntake(1);
        } else if (m_oi.getIntakeBackFeed.get()) {
            m_intake.setIntake(-0.5);
        } else {
            m_intake.setIntake(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setIntake(0);
        m_intake.setWrist(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}