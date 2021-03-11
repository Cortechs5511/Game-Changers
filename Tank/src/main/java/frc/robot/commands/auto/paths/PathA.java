package frc.robot.commands.auto.paths;

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
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

import frc.robot.subsystems.Drive;
import java.util.List;

public class PathA {

    public static RamseteCommand getPathA(Drive m_drive) {
        m_drive.resetOdometry(new Pose2d(0.762, 1.524, new Rotation2d()));
        final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics, 9);

        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics)
                        .addConstraint(autoVoltageConstraint).setReversed(false).setEndVelocity(1);

        Trajectory pathA = TrajectoryGenerator.generateTrajectory(new Pose2d(0.762, 1.524, new Rotation2d()),
                List.of(new Translation2d(2.286, 2.286), new Translation2d(4.572, 3.102),
                        new Translation2d(4.318, 0.762)),
                new Pose2d(7.112, 1.778, new Rotation2d()), config);

        RamseteCommand command = new RamseteCommand(pathA, m_drive::getPose,
                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics, m_drive::getWheelSpeeds, new PIDController(DriveConstants.kPDriveVel, 0, 0), // DriveConstants.kPDriveVel
                new PIDController(DriveConstants.kPDriveVel, 0, 0), m_drive::setOutput, m_drive);

        return command;
    }
}