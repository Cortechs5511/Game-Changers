package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANEncoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {
	private CANSparkMax left0 = new CANSparkMax(DriveConstants.kLeftMotor0Port, MotorType.kBrushless);
	private CANSparkMax left1 = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);

	private CANSparkMax right0 = new CANSparkMax(DriveConstants.kRightMotor0Port, MotorType.kBrushless);
	private CANSparkMax right1 = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);

	private CANPIDController leftNEOPID = left0.getPIDController();
	private CANPIDController rightNEOPID = right0.getPIDController();
	public PIDController anglePID = new PIDController(0.03, 0.04, 0.001);

	private CANEncoder leftEnc = left0.getEncoder();
	private CANEncoder rightEnc = right0.getEncoder();

	private AHRS navx = new AHRS();
	public Boolean invert = false;
	private double multiplier = 0.9;

	public Supplier<Double> getLeftOutput = () -> left0.get();
	public Supplier<Double> getRightOutput = () -> right0.get();

	public Supplier<Double> getLeftPosition = () -> leftEnc.getPosition();
	public Supplier<Double> getRightPosition = () -> rightEnc.getPosition();

	public Supplier<Double> getLeftVelocity = () -> leftEnc.getVelocity();
	public Supplier<Double> getRightVelocity = () -> rightEnc.getVelocity();

	public Supplier<Double> getGyroAngle = () -> navx.getAngle();
	public Supplier<Double> getMaxOutput = () -> multiplier;

	//private double angle_kP, angle_kI, angle_kD;
	private final DifferentialDriveOdometry m_odometry;

	public Drive() {
		left0.clearFaults();
		left1.clearFaults();
		right0.clearFaults();
		right1.clearFaults();

		left0.restoreFactoryDefaults();
		left1.restoreFactoryDefaults();
		right0.restoreFactoryDefaults();
		right1.restoreFactoryDefaults();

		left1.follow(left0);
		right1.follow(right0);

		left0.setIdleMode(IdleMode.kBrake);
		left1.setIdleMode(IdleMode.kBrake);
		right0.setIdleMode(IdleMode.kBrake);
		right1.setIdleMode(IdleMode.kBrake);

		left0.setInverted(false);
		left1.setInverted(false);
		right0.setInverted(true);
		right1.setInverted(true);
		// false, false, true, true

		left0.enableVoltageCompensation(9);
		left1.enableVoltageCompensation(9);
		right0.enableVoltageCompensation(9);
		right1.enableVoltageCompensation(9);

		left0.setOpenLoopRampRate(0.5);
		left1.setOpenLoopRampRate(0.5);
		right0.setOpenLoopRampRate(0.5);
		right1.setOpenLoopRampRate(0.5);

		left0.setClosedLoopRampRate(0.5); // still to be tested
		left1.setClosedLoopRampRate(0.5);
		right0.setClosedLoopRampRate(0.5);
		right1.setClosedLoopRampRate(0.5);

		left0.setSmartCurrentLimit(60, 60, 9000);
		left1.setSmartCurrentLimit(60, 60, 9000);
		right0.setSmartCurrentLimit(60, 60, 9000);
		right1.setSmartCurrentLimit(60, 60, 9000);

		leftEnc.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
		rightEnc.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);

		leftNEOPID.setP(1000);
		leftNEOPID.setI(DriveConstants.kLeftI);
		leftNEOPID.setD(DriveConstants.kLeftD);
		leftNEOPID.setFF(DriveConstants.kLeftFF);
		leftNEOPID.setOutputRange(-0.4, 0.4);

		rightNEOPID.setP(1000);
		rightNEOPID.setI(DriveConstants.kRightI);
		rightNEOPID.setD(DriveConstants.kRightD);
		rightNEOPID.setFF(DriveConstants.kRightFF);
		rightNEOPID.setOutputRange(-0.4, 0.4); // consider changing this during drive testing

		anglePID.disableContinuousInput();
		anglePID.setIntegratorRange(-1, 1);

		/*SmartDashboard.putNumber("Angle P", 0.03);
		SmartDashboard.putNumber("Angle I", 0.04);
		SmartDashboard.putNumber("Angle D", 0.001);
		SmartDashboard.putNumber("Threshold", 0.5);*/

		m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
		
		
	}

	public void setLeft(double leftInput) {
		left0.set(leftInput);
	}

	public void setRight(double rightInput) {
		right0.set(rightInput);
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(leftEnc.getVelocity(), rightEnc.getVelocity());
	}

	public double getHeading() {
		return Math.IEEEremainder(navx.getAngle(), 360);
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public void setOutput(double leftVolts, double rightVolts) { 
		left0.setVoltage(leftVolts);
		right0.setVoltage(rightVolts); 
	}

	public void resetOdometry(Pose2d pose) {
		resetLeftEnc();
		resetRightEnc();
		m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
	}

	public void setLeftPIDReference(double ref) {
		leftNEOPID.setReference(ref, ControlType.kPosition);
	}

	public void setRightPIDReference(double ref) {
		rightNEOPID.setReference(ref, ControlType.kPosition);
	}

	public void resetLeftEnc() {
		leftEnc.setPosition(0);
	}

	public void resetRightEnc() {
		rightEnc.setPosition(0);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Left Speed", leftEnc.getVelocity());
		SmartDashboard.putNumber("Right Speed", rightEnc.getVelocity());

		//SmartDashboard.putNumber("Left Position", leftEnc.getPosition());
		//SmartDashboard.putNumber("Right Position", rightEnc.getPosition());

		SmartDashboard.putNumber("Left Power", left0.getAppliedOutput());
		SmartDashboard.putNumber("Right Power", right0.getAppliedOutput());

		SmartDashboard.putNumber("NavX Angle", navx.getAngle());

		/*angle_kP = SmartDashboard.getNumber("Angle P", 0.03);
		angle_kI = SmartDashboard.getNumber("Angle I", 0.04);
		angle_kD = SmartDashboard.getNumber("Angle D", 0.001);
		anglePID.setPID(angle_kP, angle_kI, angle_kD);*/

		m_odometry.update(Rotation2d.fromDegrees(getHeading()), leftEnc.getPosition(), rightEnc.getPosition());
	}

	public boolean getDirection() {
		return invert;
	}

	public void setDirection(boolean b) {
		invert = b;
	}

	public void setMaxOutput(double limit) {
		multiplier = limit;
	}

	public void setBrake() {
		left0.setIdleMode(IdleMode.kBrake);
		left1.setIdleMode(IdleMode.kBrake);
		right0.setIdleMode(IdleMode.kBrake);
		right1.setIdleMode(IdleMode.kBrake);
	}

	public void setCoast() {
		left0.setIdleMode(IdleMode.kCoast);
		left1.setIdleMode(IdleMode.kCoast);
		right0.setIdleMode(IdleMode.kCoast);
		right1.setIdleMode(IdleMode.kCoast);
	}
}