package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drive;

public class Flip extends InstantCommand {
    private final Drive m_drive;

    public Flip(Drive drive) {
        m_drive = drive;
    }

    @Override
    public void initialize() {
        m_drive.setDirection(!(m_drive.getDirection()));
    }
}
