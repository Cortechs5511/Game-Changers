package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class SetDistance extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drive m_drive;
    private final double targetDistance;

    public SetDistance(double distance, Drive drive) {
        targetDistance = distance;
        m_drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_drive.resetLeftEnc();
        m_drive.resetRightEnc();

        m_drive.setLeftPIDReference(targetDistance);
        m_drive.setRightPIDReference(targetDistance);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setLeft(0);
        m_drive.setRight(0);
    }

    @Override
    public boolean isFinished() {
        return ((Math.abs(targetDistance - m_drive.getLeftPosition.get()) < 0.1) && (Math.abs(targetDistance - m_drive.getLeftPosition.get()) < 0.1)) || ((Math.abs(m_drive.getLeftVelocity.get()) < 30) && ((Math.abs(m_drive.getRightVelocity.get()) < 30)));
    }
}
