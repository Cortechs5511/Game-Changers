// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShootAlign;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Drive m_drive = new Drive();
    private final Intake m_intake = new Intake();
    private final Feeder m_feeder = new Feeder();
    private final Limelight m_limelight = new Limelight();
    private final Shooter m_shooter = new Shooter();

    private final SetFeederPower m_setFeederPower = new SetFeederPower(m_feeder);
    private final SetIntakePower m_setIntakePower = new SetIntakePower(m_intake, m_drive);
    private final SetSpeed m_setSpeed = new SetSpeed(m_drive);

    private final ShootAlign m_fastShootAlign = new ShootAlign(0.5, -1, m_drive, m_shooter, m_feeder, m_limelight, m_intake);
    private final ShootAlign m_slowShootAlign = new ShootAlign(0.25, 5, m_drive, m_shooter, m_feeder, m_limelight, m_intake);
    private final Shoot m_fastShoot = new Shoot(-1, m_shooter, m_feeder, m_limelight, m_intake);
    private final Shoot m_slowShoot = new Shoot(5, m_shooter, m_feeder, m_limelight, m_intake);

    private final StopShooter m_stopShooter = new StopShooter(m_shooter, m_limelight, m_feeder, m_drive);
    private final LightToggle m_lightToggle = new LightToggle(m_limelight);

    Joystick leftStick = new Joystick(0);
    Joystick rightStick = new Joystick(1);
    XboxController controller = new XboxController(2);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        m_drive.setDefaultCommand(m_setSpeed);
        m_feeder.setDefaultCommand(m_setFeederPower);
        m_intake.setDefaultCommand(m_setIntakePower);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(controller, 1).whenPressed(m_slowShoot, true);
        new JoystickButton(rightStick, 1).whenPressed(m_fastShoot, true);
        new JoystickButton(controller, 4).whenPressed(m_slowShootAlign, true);
        new JoystickButton(controller, 2).whenPressed(m_fastShootAlign, true);

        new JoystickButton(controller, 8).whenPressed(m_stopShooter, false);
        new JoystickButton(controller, 7).whenPressed(m_lightToggle, true);

        new JoystickButton(leftStick, 2).whenPressed(new Flip(m_drive));
        new JoystickButton(rightStick, 2).whenPressed(() -> m_drive.setMaxOutput(0.5))
                .whenReleased(() -> m_drive.setMaxOutput(0.9));

        new JoystickButton(rightStick, 3).whenPressed(() -> m_drive.setMaxOutput(0.25))
                .whenReleased(() -> m_drive.setMaxOutput(0.9));

        new JoystickButton(rightStick, 4).whenPressed(() -> m_drive.setMaxOutput(0.25))
                .whenReleased(() -> m_drive.setMaxOutput(0.9));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new WaitCommand(1);
    }
}
