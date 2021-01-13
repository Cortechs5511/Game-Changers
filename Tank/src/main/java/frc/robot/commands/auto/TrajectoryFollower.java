package frc.robot.commands.auto;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

import frc.robot.subsystems.Drive;

public class TrajectoryFollower extends CommandBase {
	private Drive m_drive;

	public TrajectoryFollower(Drive drive) {
		m_drive = drive;
		addRequirements(m_drive);
	}

	public RamseteCommand getPath(String trajectoryJSON) {
		DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
						DriveConstants.kaVoltSecondsSquaredPerMeter),
				DriveConstants.kDriveKinematics, 9);

		TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
				AutoConstants.kMaxAccelerationMetersPerSecondSquared);
				
		config.setKinematics(DriveConstants.kDriveKinematics);
		config.addConstraint(autoVoltageConstraint);

		try {
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
			Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

			System.out.println(trajectory.getStates());

			RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_drive::getPose,
					new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
					new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
							DriveConstants.kaVoltSecondsSquaredPerMeter),
					DriveConstants.kDriveKinematics, m_drive::getWheelSpeeds,
					new PIDController(DriveConstants.kPDriveVel, 0, 0),
					new PIDController(DriveConstants.kPDriveVel, 0, 0),
					// RamseteCommand passes volts to the callback
					m_drive::setOutput, m_drive);
			return ramseteCommand;
		} catch (IOException ex) {
			System.out.println("Unable to open trajectory: " + trajectoryJSON);
		}
		return null;
	}

}
