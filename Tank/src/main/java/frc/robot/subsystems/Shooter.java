package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
    private final CANSparkMax shoot0 = new CANSparkMax(ShooterConstants.kShoot0Port, MotorType.kBrushless);
    private final CANSparkMax shoot1 = new CANSparkMax(ShooterConstants.kShoot1Port, MotorType.kBrushless);

    private final CANPIDController shootPID = shoot0.getPIDController();
    private final CANEncoder shootEnc = shoot0.getEncoder();
    public Supplier<Double> getSpeed = shootEnc::getVelocity;
    private double reference = 0f;
    public Supplier<Double> getTarget = () -> reference;

    public Shooter() {
        shoot0.restoreFactoryDefaults();
        shoot1.restoreFactoryDefaults();

        shoot0.setIdleMode(IdleMode.kCoast);
        shoot1.setIdleMode(IdleMode.kCoast);

        shoot0.enableVoltageCompensation(11);
        shoot1.enableVoltageCompensation(11);

        shoot0.setSecondaryCurrentLimit(250);
        shoot1.setSecondaryCurrentLimit(250);

        shoot0.setSmartCurrentLimit(200, 200, 200000);
        shoot1.setSmartCurrentLimit(200, 200, 200000);

        shoot0.setClosedLoopRampRate(1.5);
        shoot1.setClosedLoopRampRate(1.5);

        shoot0.setOpenLoopRampRate(1.5);
        shoot1.setOpenLoopRampRate(1.5);

        shoot0.setInverted(true);
        shoot1.setInverted(true);

        shoot1.follow(shoot0, true);

        shootPID.setOutputRange(0, 1);

        shootPID.setP(ShooterConstants.kShootP);
        shootPID.setI(ShooterConstants.kShootI);
        shootPID.setD(ShooterConstants.kShootD);
        shootPID.setFF(ShooterConstants.kShootFF);

        SmartDashboard.putNumber("Shooter/Shooter P", ShooterConstants.kShootP);
        SmartDashboard.putNumber("Shooter/Shooter I", ShooterConstants.kShootI);
        SmartDashboard.putNumber("Shooter/Shooter D", ShooterConstants.kShootD);
        SmartDashboard.putNumber("Shooter/Shooter FF", ShooterConstants.kShootFF);

        SmartDashboard.putNumber("Shooter/RPM Adjustment", 0);
    }

    public void setSpeed(double input) {
        shoot0.set(input);
        shoot1.set(input);
    }

    public void setRampRate(double rate) {
        shoot0.setClosedLoopRampRate(rate);
        shoot1.setClosedLoopRampRate(rate);
    }

    public void setPIDReference(double ref) {
        reference = ref;
        shootPID.setReference(ref, ControlType.kVelocity);
    }

    @Override
    public void periodic() {
        shootPID.setP(SmartDashboard.getNumber("Shooter/Shooter P", ShooterConstants.kShootP));
        shootPID.setI(SmartDashboard.getNumber("Shooter/Shooter I", ShooterConstants.kShootI));
        shootPID.setD(SmartDashboard.getNumber("Shooter/Shooter D", ShooterConstants.kShootD));
        shootPID.setFF(SmartDashboard.getNumber("Shooter/Shooter FF", ShooterConstants.kShootFF));

        SmartDashboard.putNumber("Shooter/Shooter RPM", shootEnc.getVelocity());
        SmartDashboard.putNumber("Shooter/Shooter Output", shoot0.get());
    }
}