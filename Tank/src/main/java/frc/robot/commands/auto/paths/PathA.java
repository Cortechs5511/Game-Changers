package frc.robot.commands.auto.paths;

import java.util.Arrays;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
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
import java.util.List;

public class PathA extends CommandBase {
	private Drive m_drive;

	public PathA(Drive drive) {
		m_drive = drive;
		addRequirements(drive);
	}
    
	public RamseteCommand getPathA() {
		final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
						DriveConstants.kaVoltSecondsSquaredPerMeter),
				DriveConstants.kDriveKinematics, 9);

		TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
				AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics)
						.addConstraint(autoVoltageConstraint);

		Trajectory pathA = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d()),
            List.of(
            new Translation2d(2.2098, -0.508),
            new Translation2d(3.7338, -1.27),
            new Translation2d(4.4958, 1.016),
            new Translation2d(6.0198, 0.254)),
            new Pose2d(10, 0.254, new Rotation2d()),
            config);

		RamseteCommand command = new RamseteCommand(pathA, m_drive::getPose,
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