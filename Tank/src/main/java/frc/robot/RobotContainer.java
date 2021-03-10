package frc.robot;

import frc.robot.commands.*;
import frc.robot.commands.auto.TrajectoryFollower;
import frc.robot.commands.auto.paths.*;
/*import frc.robot.commands.auto.groups.BackTowerSimple;
import frc.robot.commands.auto.groups.TowerSimple;
import frc.robot.commands.auto.groups.TowerSimpleForward;
import frc.robot.commands.auto.groups.TrenchSimple;*/
import frc.robot.commands.drive.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.climber.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class RobotContainer {
    private final Drive m_drive = new Drive();
    private final Intake m_intake = new Intake();
    private final Feeder m_feeder = new Feeder();
    private final Limelight m_limelight = new Limelight();
    private final Shooter m_shooter = new Shooter();
    private final Climber m_climber = new Climber();

    private final SetFeederPower m_setFeederPower = new SetFeederPower(m_feeder);
    private final SetIntakePower m_setIntakePower = new SetIntakePower(m_intake, m_drive);
    private final AutoCollect m_autoCollect = new AutoCollect(m_intake, m_feeder);
    private final ManualClimb m_manualClimb = new ManualClimb(m_climber);
    private final SetSpeed m_setSpeed = new SetSpeed(m_drive);

    // 0.5 degree threshold slowShootAlign prev
    private final ShootAlign m_fastShootAlign = new ShootAlign(0.5, -1, m_drive, m_shooter, m_feeder, m_limelight,
            m_intake);
    private final ShootAlign m_slowShootAlign = new ShootAlign(0.25, 5, m_drive, m_shooter, m_feeder, m_limelight,
            m_intake);
    private final Shoot m_fastShoot = new Shoot(-1, m_shooter, m_feeder, m_limelight, m_intake);
    private final Shoot m_slowShoot = new Shoot(5, m_shooter, m_feeder, m_limelight, m_intake);

    private final StopShooter m_stopShooter = new StopShooter(m_shooter, m_limelight, m_feeder, m_drive);
    private final LightToggle m_lightToggle = new LightToggle(m_limelight);

    Joystick leftStick = new Joystick(0);
    Joystick rightStick = new Joystick(1);
    XboxController controller = new XboxController(2);

    enum autonMode {
        PathA, TowerSimple, Turn90
    }

    SendableChooser<autonMode> m_chooser = new SendableChooser<>();

    public static double getNumber(String key, double defaultValue) {
        if (SmartDashboard.containsKey(key)) {
            return SmartDashboard.getNumber(key, defaultValue);
        }
        SmartDashboard.putNumber(key, defaultValue);
        return defaultValue;
    }

    public RobotContainer() {
        configureButtonBindings();

        m_drive.setDefaultCommand(m_setSpeed);
        m_feeder.setDefaultCommand(m_setFeederPower);
        m_intake.setDefaultCommand(m_setIntakePower);
        m_climber.setDefaultCommand(m_manualClimb);

        getNumber("DriveDelay", 3.0);
        getNumber("ShootDelay", 3.0);

        m_chooser.addOption("Tower Simple", autonMode.TowerSimple);
        // m_chooser.addOption("Trench Simple", autonMode.TrenchSimple);

        m_chooser.addOption("Path A", autonMode.PathA);
        m_chooser.addOption("Turn 90", autonMode.Turn90);
        m_chooser.setDefaultOption("Turn 90", autonMode.Turn90);

        Shuffleboard.getTab("Autonomous").add(m_chooser);
    }

    private void configureButtonBindings() {
        new JoystickButton(controller, 1).whenPressed(m_slowShoot, true);
        new JoystickButton(controller, 3).whenPressed(m_fastShoot, true);
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

    public Command getAutonomousCommand() {
        m_drive.resetOdometry(new Pose2d());
		switch (m_chooser.getSelected()) {
		case TowerSimple:
			return TowerSimple.getTowerSimple(m_drive);
		/*case TrenchSimple:
			return new WaitCommand(1.0);*/
		case PathA:
            return m_autoCollect.andThen(TrajectoryFollower.getPath("paths/Red.path", m_drive).andThen(stop()));
		case Turn90:
			return Turn90.getTurn90(m_drive).andThen(new WaitCommand(5)).andThen(stop());
		default:
			return new WaitCommand(1.0);

		}
	}

	private Command stop() {
        return new StopDrive(m_drive);
    }


    public void teleopInit(Robot robot) {
		new Coast(m_drive);
		if (robot.m_autonomousCommand != null) {
			robot.m_autonomousCommand.cancel();
		}
	}
}