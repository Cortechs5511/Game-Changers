package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.TrajectoryFollower;
import frc.robot.commands.drive.Flip;
import frc.robot.commands.drive.SetSpeed;
import frc.robot.commands.drive.StopDrive;
import frc.robot.subsystems.Drive;

public class RobotContainer {
    private final Drive m_drive = new Drive();

    Joystick leftStick = new Joystick(0);
    Joystick rightStick = new Joystick(1);
    SendableChooser<autonMode> m_chooser = new SendableChooser<>();

    enum autonMode {
        Testing, BounceTest, BouncePath, CurveTest, Slalom, BarrelRacing
    }

    public RobotContainer() {
        configureButtonBindings();

        SetSpeed m_setSpeed = new SetSpeed(m_drive);
        m_drive.setDefaultCommand(m_setSpeed);

        m_chooser.addOption("Testing", autonMode.Testing);
        m_chooser.addOption("Bounce Test", autonMode.BounceTest);
        m_chooser.addOption("Bounce Path", autonMode.BouncePath);
        m_chooser.addOption("Slalom", autonMode.Slalom);
        m_chooser.addOption("Barrel Racing", autonMode.BarrelRacing);
        m_chooser.setDefaultOption("Curve Test", autonMode.CurveTest);

        Shuffleboard.getTab("Autonomous").add(m_chooser);
        // getAutonomousCommand(autonMode.BarrelRacing);
        // getAutonomousCommand(autonMode.AutoNavB);
        // getAutonomousCommand(autonMode.Slalom);
    }


    private void configureButtonBindings() {
        new JoystickButton(leftStick, 2).whenPressed(new Flip(m_drive));
        new JoystickButton(rightStick, 2).whenPressed(() -> m_drive.setMaxOutput(0.5))
                .whenReleased(() -> m_drive.setMaxOutput(0.9));

        new JoystickButton(rightStick, 3).whenPressed(() -> m_drive.setMaxOutput(0.25))
                .whenReleased(() -> m_drive.setMaxOutput(0.9));

        new JoystickButton(rightStick, 4).whenPressed(() -> m_drive.setMaxOutput(0.25))
                .whenReleased(() -> m_drive.setMaxOutput(0.9));
    }

    public Command getAutonomousCommand(autonMode choice) {
        switch (choice) {
            case Testing:
                return TrajectoryFollower.getPath("output/Testing.wpilib.json", m_drive, true).andThen(stop());
            case BounceTest:
                return TrajectoryFollower.getPath("output/BounceTest1.wpilib.json", m_drive, true)
                        .andThen(TrajectoryFollower.getPath("output/BounceTest2.wpilib.json", m_drive, false));
            case BouncePath:
                return TrajectoryFollower.getPath("output/Bounce1.wpilib.json", m_drive, true)
                .andThen(TrajectoryFollower.getPath("output/Bounce2.wpilib.json", m_drive, false)
                .andThen(TrajectoryFollower.getPath("output/Bounce3.wpilib.json", m_drive, false)
                .andThen(TrajectoryFollower.getPath("output/Bounce4.wpilib.json", m_drive, false).andThen(stop()))));
            case Slalom:
                return TrajectoryFollower.getPath("output/Slalom.wpilib.json", m_drive, true).andThen(stop());
            case BarrelRacing:
                return TrajectoryFollower.getPath("output/BarrelRacing.wpilib.json", m_drive, true).andThen(stop());
            case CurveTest:
                return TrajectoryFollower.getPath("output/FirstCurve.wpilib.json", m_drive, true)
                        .andThen(TrajectoryFollower.getPath("output/SecondCurve.wpilib.json", m_drive, false));
            default:
                return new WaitCommand(1.0);
        }
    }

    public Command getAutonomousCommand() {
        return getAutonomousCommand(m_chooser.getSelected());
    }

    private Command stop() {
        return new StopDrive(m_drive);
    }

    public void teleopInit(Robot robot) {
        m_drive.resetLeftEnc();
        m_drive.resetRightEnc();
        if (robot.m_autonomousCommand != null) {
            robot.m_autonomousCommand.cancel();
        }
    }

    public void disabledInit() {
        m_drive.setLeft(0);
        m_drive.setRight(0);
    }
}