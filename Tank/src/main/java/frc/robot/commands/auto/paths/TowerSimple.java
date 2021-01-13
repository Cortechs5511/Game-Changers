package frc.robot.commands.auto.paths;

import java.util.Arrays;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

import frc.robot.subsystems.Drive;

public class TowerSimple extends CommandBase {
	private Drive m_drive;

	public TowerSimple(Drive drive) {
		m_drive = drive;
		addRequirements(drive);
	}

	public RamseteCommand getTowerSimple() {
		final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
						DriveConstants.kaVoltSecondsSquaredPerMeter),
				DriveConstants.kDriveKinematics, 9);

		TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
				AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics)
						.addConstraint(autoVoltageConstraint);

		Trajectory towerSimple = TrajectoryGenerator.generateTrajectory(Arrays.asList(
			new Pose2d(),
			new Pose2d(3.0, -2.404364, new Rotation2d()), 
			new Pose2d(4.5, -2.404, new Rotation2d())), 
			config);

		RamseteCommand command = new RamseteCommand(towerSimple, m_drive::getPose,
				new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
				new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
						DriveConstants.kaVoltSecondsSquaredPerMeter),
				DriveConstants.kDriveKinematics, 
				m_drive::getWheelSpeeds,
				new PIDController(DriveConstants.kPDriveVel, 0, 0), 
				new PIDController(DriveConstants.kPDriveVel, 0, 0),
				m_drive::setOutput, m_drive);

		return command;
	}

}
