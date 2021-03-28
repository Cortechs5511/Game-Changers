package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import java.util.function.Supplier;

public class Drive extends SubsystemBase {
    public final static double MAX_ALLOWED_VOLTS = 9.0;

    private final CANSparkMax left0 = createSparkMAX(DriveConstants.kLeftMotor0Port, false);
    private final CANSparkMax right0 = createSparkMAX(DriveConstants.kRightMotor0Port, true);

    private final CANSparkMax left1 = createSparkMAX(DriveConstants.kLeftMotor1Port, false);
    private final CANSparkMax right1 = createSparkMAX(DriveConstants.kRightMotor1Port, true);

    private final CANEncoder leftEnc = left0.getEncoder();
    private final CANEncoder rightEnc = right0.getEncoder();
    private final AHRS navx = new AHRS();
    private final DifferentialDriveOdometry m_odometry;
    private final Field2d m_fieldSim = new Field2d();
    public Boolean invert = false;
    public Supplier<Double> getLeftOutput = left0::get;
    public Supplier<Double> getRightOutput = right0::get;
    public Supplier<Double> getLeftVelocity = leftEnc::getVelocity;
    public Supplier<Double> getRightVelocity = rightEnc::getVelocity;
    public Supplier<Double> getGyroAngle = navx::getAngle;

    private double multiplier = 0.9;
    public Supplier<Double> getMaxOutput = () -> multiplier;

    public Drive() {
        left1.follow(left0);
        right1.follow(right0);

        leftEnc.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
        rightEnc.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);

        leftEnc.setVelocityConversionFactor((DriveConstants.kWheelDiameterMeters * Math.PI)
                / (DriveConstants.kGearing * DriveConstants.kEncoderCPR));
        rightEnc.setVelocityConversionFactor((DriveConstants.kWheelDiameterMeters * Math.PI)
                / (DriveConstants.kGearing * DriveConstants.kEncoderCPR));

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        SmartDashboard.putData("Field", m_fieldSim);
    }
    
    private CANSparkMax createSparkMAX(int id, boolean invert) {
        CANSparkMax controller = new CANSparkMax(id, MotorType.kBrushless);

        controller.clearFaults();
        controller.restoreFactoryDefaults();
        controller.setIdleMode(IdleMode.kBrake);
        controller.setInverted(invert);
        controller.enableVoltageCompensation(MAX_ALLOWED_VOLTS);
        controller.setOpenLoopRampRate(0.1);
        controller.setClosedLoopRampRate(0.1);
        controller.setSmartCurrentLimit(60, 60, 9000);

        return controller;
    }


    public void setLeft(double leftInput) {
        left0.set(leftInput);
    }

    public void setRight(double rightInput) {
        right0.set(rightInput);
    }

    public double getHeading() {
        return -Math.IEEEremainder(navx.getAngle(), 360);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Set the voltage output for the left and right side of the robot drive.
     * 
     * <p>Use positive values to move forward. If you give positive values and it
     * doesn't move forward then this method is broken and I'll blame the engineers.</p>
     * 
     * @param leftVolts Voltage output for left side in the range of +/- {@link #MAX_ALLOWED_VOLTS}.
     * @param rightVolts Voltage output for right side in the range of +/- {@link #MAX_ALLOWED_VOLTS}.
     */
    public void setOutput(double leftVolts, double rightVolts) {
        left0.set(rightVolts / MAX_ALLOWED_VOLTS);
        right0.set(leftVolts / MAX_ALLOWED_VOLTS);
    }

    /**
     * Reset the encoders and navx to 0.0 as well as current pose.
     * 
     * @param pose Pose to reset - must not be null.
     */
    public void resetOdometry(Pose2d pose) {
        resetLeftEnc();
        resetRightEnc();
        navx.reset();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    public void resetLeftEnc() {
        leftEnc.setPosition(0);
    }

    public void resetRightEnc() {
        rightEnc.setPosition(0);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEnc.getVelocity(), rightEnc.getVelocity());
    }

    @Override
    public void periodic() {
        double leftVelocity = leftEnc.getVelocity();
        double rightVelocity = rightEnc.getVelocity();

        SmartDashboard.putNumber("Drive/Left Speed", leftVelocity);
        SmartDashboard.putNumber("Drive/Right Speed", rightVelocity);
 
        double leftPosition = leftEnc.getPosition();
        double rightPosition = rightEnc.getPosition();

        SmartDashboard.putNumber("Drive/Left Position", leftPosition);
        SmartDashboard.putNumber("Drive/Right Position", rightPosition);

        SmartDashboard.putNumber("Drive/Left Power", left0.getAppliedOutput());
        SmartDashboard.putNumber("Drive/Right Power", right0.getAppliedOutput());

        SmartDashboard.putNumber("NavX Angle", navx.getAngle());

        m_odometry.update(Rotation2d.fromDegrees(getHeading()), leftEnc.getPosition(), rightEnc.getPosition());
    }

    public void setMaxOutput(double limit) {
        multiplier = limit;
    }
}