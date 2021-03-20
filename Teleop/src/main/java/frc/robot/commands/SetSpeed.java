// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drive;
import frc.robot.OI;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetSpeed extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final OI m_oi = OI.getInstance();
	private final Drive m_drive;

	public SetSpeed(Drive drive) {
		m_drive = drive;
		addRequirements(drive);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		double leftInput = m_oi.getLeftY.get();
		double rightInput = m_oi.getRightY.get();

		double leftOutput;
		if (Math.abs(leftInput) > 0.1) {
			leftOutput = leftInput * m_drive.maxOutput;
		} else {
			leftOutput = 0;
		}
		double rightOutput;
		if (Math.abs(rightInput) > 0.1) {
			rightOutput = rightInput * m_drive.maxOutput;
		} else {
			rightOutput = 0;
		}

		if (!m_drive.invert) {
			m_drive.set(leftOutput, rightOutput);
		} else {
			m_drive.set(-rightOutput, -leftOutput);
		}
	}

	@Override
	public void end(boolean interrupted) {
		m_drive.set(0, 0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
