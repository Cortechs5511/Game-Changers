package frc.robot;

import frc.robot.commands.*;
import frc.robot.commands.auto.TrajectoryFollower;
import frc.robot.commands.auto.paths.*;
import frc.robot.commands.drive.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class RobotContainer {
    private final Drive m_drive = new Drive();
    private final Intake m_intake = new Intake();
    private final Feeder m_feeder = new Feeder();

    Joystick leftStick = new Joystick(0);
    Joystick rightStick = new Joystick(1);

    enum autonMode {
        GalacticSearch, TowerSimple, Turn90, Turn180, Testing, BounceTest, AutoNavB, CurveTest, Slalom, BarrelRacing
    }

    SendableChooser<autonMode> m_chooser = new SendableChooser<>();

    public RobotContainer() {
        configureButtonBindings();

        SetSpeed m_setSpeed = new SetSpeed(m_drive);
        m_drive.setDefaultCommand(m_setSpeed);
        SetFeederPower m_setFeederPower = new SetFeederPower(m_feeder);
        m_feeder.setDefaultCommand(m_setFeederPower);
        SetIntakePower m_setIntakePower = new SetIntakePower(m_intake, m_drive);
        m_intake.setDefaultCommand(m_setIntakePower);

        m_chooser.addOption("Tower Simple", autonMode.TowerSimple);
        // m_chooser.addOption("Trench Simple", autonMode.TrenchSimple);
        m_chooser.addOption("Galactic Search", autonMode.GalacticSearch);
        m_chooser.addOption("Testing", autonMode.Testing);
        m_chooser.addOption("Turn 90", autonMode.Turn90);
        m_chooser.addOption("Turn 180", autonMode.Turn180);
        m_chooser.addOption("Bounce Test", autonMode.BounceTest);
        m_chooser.addOption("AutoNav B", autonMode.AutoNavB);
        m_chooser.addOption("Slalom", autonMode.Slalom);
        m_chooser.addOption("Barrel Racing", autonMode.BarrelRacing);
        m_chooser.setDefaultOption("Curve Test", autonMode.CurveTest);

        Shuffleboard.getTab("Autonomous").add(m_chooser);
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

    public Command getAutonomousCommand() {
        switch (m_chooser.getSelected()) {
        case TowerSimple:
            return TowerSimple.getTowerSimple(m_drive);
        case GalacticSearch:
            return new AutoCollect(m_intake, m_feeder).alongWith(
                    TrajectoryFollower.getPath("output/GalacticSearch.wpilib.json", m_drive, true).andThen(stop()));
        case Testing:
            return TrajectoryFollower.getPath("output/Testing.wpilib.json", m_drive, true).andThen(stop());
        case BounceTest:
            return TrajectoryFollower.getPath("output/BounceTest1.wpilib.json", m_drive, true)
                    .andThen(TrajectoryFollower.getPath("output/BounceTest2.wpilib.json", m_drive, false));
        case Turn90:
            return new AutoCollect(m_intake, m_feeder)
                    .alongWith(Turn90.getTurn90(m_drive).andThen(new WaitCommand(5)).andThen(stop()));
        case Turn180:
            return new AutoCollect(m_intake, m_feeder)
                    .alongWith(Turn180.getTurn180(m_drive).andThen(new WaitCommand(5)).andThen(stop()));
        case AutoNavB:
            return TrajectoryFollower.getPath("output/AutoNavB.wpilib.json", m_drive, true).andThen(stop());
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