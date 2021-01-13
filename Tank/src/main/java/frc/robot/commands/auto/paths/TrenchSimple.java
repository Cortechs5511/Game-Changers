package frc.robot.commands.auto.paths;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

import frc.robot.subsystems.Drive;

public class TrenchSimple extends CommandBase {
	private Drive m_drive;

	public TrenchSimple(Drive drive) {
		m_drive = drive;
		addRequirements(drive);
	}

	public RamseteCommand getTrenchSimple() {
		final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
						DriveConstants.kaVoltSecondsSquaredPerMeter),
				DriveConstants.kDriveKinematics, 9);

		TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
				AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics)
						.addConstraint(autoVoltageConstraint);

		Trajectory trenchSimple = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				new Pose2d(0, 0, new Rotation2d(Math.toRadians(20))), List.of(new Translation2d(-1.1, -0.4)),
				new Pose2d(-2.2, -0.70485, new Rotation2d(0)),
				// Pass config
				config);

		RamseteCommand ramseteCommand = new RamseteCommand(trenchSimple, m_drive::getPose,
				new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
				new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
						DriveConstants.kaVoltSecondsSquaredPerMeter),
				DriveConstants.kDriveKinematics, m_drive::getWheelSpeeds,
				new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
				// RamseteCommand passes volts to the callback
				m_drive::setOutput, m_drive);

		return ramseteCommand;
	}
}
