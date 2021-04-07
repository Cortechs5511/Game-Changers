// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final CANSparkMax shoot0 = createSparkMAX(ShooterConstants.kShoot0Port);
    private final CANSparkMax shoot1 = createSparkMAX(ShooterConstants.kShoot1Port);

    private final CANPIDController shootPID = shoot0.getPIDController();
    public final CANEncoder shootEncoder = shoot0.getEncoder();

    private double reference = 0.0;
    public Supplier<Double> getSpeed = shootEncoder::getVelocity;
    public Supplier<Double> getReference = () -> reference;

    public Shooter() {
        shoot1.follow(shoot0, true);
        shootPID.setOutputRange(0, 1);

        shootPID.setP(ShooterConstants.kShootP);
        shootPID.setI(ShooterConstants.kShootI);
        shootPID.setD(ShooterConstants.kShootD);
        shootPID.setFF(ShooterConstants.kShootFF);
    }


    private CANSparkMax createSparkMAX(int id) {
        CANSparkMax controller = new CANSparkMax(id, MotorType.kBrushless);

        controller.clearFaults();
        controller.restoreFactoryDefaults();
        controller.setIdleMode(IdleMode.kCoast);
        controller.setInverted(true);

        controller.enableVoltageCompensation(11);
        controller.setOpenLoopRampRate(0.75);
        controller.setClosedLoopRampRate(0.75);
        controller.setSmartCurrentLimit(80, 80, 9000);
        controller.setSecondaryCurrentLimit(200);

        return controller;
    }

    public void setOutput(double x) {
        shoot0.set(x);
        shoot1.set(x);
    }

    public void setRampRate(double rate) {
        shoot0.setClosedLoopRampRate(rate);
        shoot1.setClosedLoopRampRate(rate);
    }

    public void setPIDReference(double ref) {
        shootPID.setReference(ref, ControlType.kVelocity);
        reference = ref;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Shooter RPM", shootEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter/Shooter Output", shoot0.get());
    }
}
