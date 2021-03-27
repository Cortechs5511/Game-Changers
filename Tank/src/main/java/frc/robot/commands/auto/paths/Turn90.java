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

public class Turn90 {
    
	public static RamseteCommand getTurn90(Drive m_drive) {
		m_drive.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
		final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(DriveConstants.ksVolts * 15, DriveConstants.kvVoltSecondsPerMeter,
						DriveConstants.kaVoltSecondsSquaredPerMeter),
				DriveConstants.kDriveKinematics, 9);

		TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
				AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics)
						.addConstraint(autoVoltageConstraint);

		Trajectory turn90 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d()),
            List.of(
			
			
			/*new Translation2d(2.286, 0.762),
			//new Translation2d(3.81, 1.524),
			new Translation2d(3.81, 1.42),
			//new Translation2d(4.572, -0.762),
			new Translation2d(4.37, -0.762),
			new Translation2d(6.096, 0.3)),
			new Pose2d(10.16, 1.524, new Rotation2d()),
			*/
			

			new Translation2d(-0.5, -0.5)),
			new Pose2d(-1, -2, new Rotation2d()),
			


            config); // trajectory ends here

		RamseteCommand command = new RamseteCommand(turn90, m_drive::getPose,
				new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
				new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
						DriveConstants.kaVoltSecondsSquaredPerMeter),
				DriveConstants.kDriveKinematics, 
				m_drive::getWheelSpeeds,
				new PIDController(DriveConstants.kPDriveVel, 0, 0), //DriveConstants.kPDriveVel
				new PIDController(DriveConstants.kPDriveVel, 0, 0),
				m_drive::setOutput, m_drive);

		return command;
	}
}