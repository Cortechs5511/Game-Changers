// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.LightToggle;
import frc.robot.commands.SetFeederPower;
import frc.robot.commands.SetIntakePower;
import frc.robot.commands.SetSpeed;
import frc.robot.commands.shooter.ShootAlign;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    private final Drive m_drive = new Drive();
    private final Shooter m_shooter = new Shooter();
    private final Feeder m_feeder = new Feeder();
    private final Limelight m_limelight = new Limelight();
    private final Intake m_intake = new Intake();

    private final ShootAlign m_shootAlignFast = new ShootAlign(0.5, -1, m_drive, m_shooter, m_feeder, m_limelight, m_intake);
    private final ShootAlign m_shootAlignSlow = new ShootAlign(0.1, 50, m_drive, m_shooter, m_feeder, m_limelight, m_intake);

    private final StopShooter m_stopShooter = new StopShooter(m_shooter, m_limelight, m_feeder, m_drive);
    private final LightToggle m_lightToggle = new LightToggle(m_limelight);

    Joystick leftStick = new Joystick(0);
    Joystick rightStick = new Joystick(1);
    XboxController controller = new XboxController(2);

    public RobotContainer() {
        configureButtonBindings();

        SetSpeed m_setSpeed = new SetSpeed(m_drive);
        SetFeederPower m_setFeederPower = new SetFeederPower(m_feeder);
        SetIntakePower m_setIntakePower = new SetIntakePower(m_intake);

        m_drive.setDefaultCommand(m_setSpeed);
        m_feeder.setDefaultCommand(m_setFeederPower);
        m_intake.setDefaultCommand(m_setIntakePower);
    }

    private void configureButtonBindings() {
        new JoystickButton(leftStick, 2).whenPressed(m_drive::flip);
        new JoystickButton(rightStick, 2).whenPressed(() -> m_drive.setMaxOutput(0.5)).whenReleased(() -> m_drive.setMaxOutput(1.0));
        new JoystickButton(rightStick, 4).whenPressed(() -> m_drive.setMaxOutput(0.25)).whenReleased(() -> m_drive.setMaxOutput(1.0));
        
        new JoystickButton(controller, 4).whenPressed(m_shootAlignSlow, true);
        new JoystickButton(controller, 2).whenPressed(m_shootAlignFast, true);
        new JoystickButton(controller, 3).whenPressed((new StopShooter(m_shooter, m_limelight, m_feeder, m_drive)).andThen(() -> m_shooter.setOutput(ShooterConstants.kIdlePower)));
        new JoystickButton(controller, 8).whenPressed(m_stopShooter, false);
        new JoystickButton(controller, 7).whenPressed(m_lightToggle, true);
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
