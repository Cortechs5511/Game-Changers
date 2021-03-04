package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import java.util.function.Supplier;

public class Drive extends SubsystemBase {
    private final CANSparkMax left0 = new CANSparkMax(DriveConstants.kLeftMotor0Port, MotorType.kBrushless);
    private final CANSparkMax left1 = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);

    private final CANSparkMax right0 = new CANSparkMax(DriveConstants.kRightMotor0Port, MotorType.kBrushless);
    private final CANSparkMax right1 = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);

    private final CANEncoder leftEnc = left0.getEncoder();
    private final CANEncoder rightEnc = right0.getEncoder();
    private final AHRS navx = new AHRS();

    public PIDController anglePID = new PIDController(DriveConstants.kAngleP, DriveConstants.kAngleI, DriveConstants.kAngleD);
    public Boolean invert = false;
    public Supplier<Double> getLeftVelocity = leftEnc::getVelocity;
    public Supplier<Double> getRightVelocity = rightEnc::getVelocity;
    private double multiplier = 0.9;
    public Supplier<Double> getMaxOutput = () -> multiplier;


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

        left0.setIdleMode(IdleMode.kCoast);
        left1.setIdleMode(IdleMode.kCoast);
        right0.setIdleMode(IdleMode.kCoast);
        right1.setIdleMode(IdleMode.kCoast);

        left0.setInverted(false);
        left1.setInverted(false);
        right0.setInverted(true);
        right1.setInverted(true);

        left0.enableVoltageCompensation(9);
        left1.enableVoltageCompensation(9);
        right0.enableVoltageCompensation(9);
        right1.enableVoltageCompensation(9);

        left0.setOpenLoopRampRate(1);
        left1.setOpenLoopRampRate(1);
        right0.setOpenLoopRampRate(1);
        right1.setOpenLoopRampRate(1);

        left0.setClosedLoopRampRate(1);
        left1.setClosedLoopRampRate(1);
        right0.setClosedLoopRampRate(1);
        right1.setClosedLoopRampRate(1);

        left0.setSmartCurrentLimit(60, 60, 9000);
        left1.setSmartCurrentLimit(60, 60, 9000);
        right0.setSmartCurrentLimit(60, 60, 9000);
        right1.setSmartCurrentLimit(60, 60, 9000);

        leftEnc.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
        rightEnc.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);

        CANPIDController leftNEOPID = left0.getPIDController();
        leftNEOPID.setP(DriveConstants.kLeftP);
        leftNEOPID.setI(DriveConstants.kLeftI);
        leftNEOPID.setD(DriveConstants.kLeftD);
        leftNEOPID.setFF(DriveConstants.kLeftFF);
        leftNEOPID.setOutputRange(-0.4, 0.4);

        CANPIDController rightNEOPID = right0.getPIDController();
        rightNEOPID.setP(DriveConstants.kRightP);
        rightNEOPID.setI(DriveConstants.kRightI);
        rightNEOPID.setD(DriveConstants.kRightD);
        rightNEOPID.setFF(DriveConstants.kRightFF);
        rightNEOPID.setOutputRange(-0.4, 0.4); // consider changing this during drive testing

        anglePID.disableContinuousInput();
        anglePID.setIntegratorRange(-1, 1);

        SmartDashboard.putNumber("Drive/Angle P", 0.03);
        SmartDashboard.putNumber("Drive/Angle I", 0.04);
        SmartDashboard.putNumber("Drive/Angle D", 0.001);
        SmartDashboard.putNumber("Drive/Threshold", 0.5);
    }

    public void setLeft(double leftInput) {
        left0.set(leftInput);
    }

    public void setRight(double rightInput) {
        right0.set(rightInput);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Drive/Left Speed", leftEnc.getVelocity());
        SmartDashboard.putNumber("Drive/Right Speed", rightEnc.getVelocity());

        SmartDashboard.putNumber("Drive/Left Position", leftEnc.getPosition());
        SmartDashboard.putNumber("Drive/Right Position", rightEnc.getPosition());

        SmartDashboard.putNumber("Drive/Left Power", left0.getAppliedOutput());
        SmartDashboard.putNumber("Drive/Right Power", right0.getAppliedOutput());

        SmartDashboard.putNumber("Drive/NavX Angle", navx.getAngle());

        if (SmartDashboard.getBoolean("Tuning Mode", false)) {
            double angle_kP = SmartDashboard.getNumber("Drive/Angle P", 0.03);
            double angle_kI = SmartDashboard.getNumber("Drive/Angle I", 0.04);
            double angle_kD = SmartDashboard.getNumber("Drive/Angle D", 0.001);
            anglePID.setPID(angle_kP, angle_kI, angle_kD);
        }
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
}