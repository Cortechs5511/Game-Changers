// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import java.util.function.Supplier;

public class Drive extends SubsystemBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final CANSparkMax left0 = createSparkMAX(DriveConstants.kLeftMotor0Port, false);
    private final CANSparkMax right0 = createSparkMAX(DriveConstants.kRightMotor0Port, true);

    private final CANEncoder leftEnc = left0.getEncoder();
    private final CANEncoder rightEnc = right0.getEncoder();

    public Boolean invert = false;
    public double maxOutput = 1.0;
    public PIDController anglePID = new PIDController(DriveConstants.kAngleP, DriveConstants.kAngleI, DriveConstants.kAngleD);

    public Supplier<Double> getLeftVelocity = leftEnc::getVelocity;
    public Supplier<Double> getRightVelocity = rightEnc::getVelocity;

    public Drive() {
        CANSparkMax left1 = createSparkMAX(DriveConstants.kLeftMotor1Port, false);
        CANSparkMax right1 = createSparkMAX(DriveConstants.kRightMotor1Port, true);

        left1.follow(left0);
        right1.follow(right0);

        anglePID.disableContinuousInput();
        anglePID.setIntegratorRange(-0.1, 0.1);
    }

    private CANSparkMax createSparkMAX(int id, boolean invert) {
        CANSparkMax controller = new CANSparkMax(id, MotorType.kBrushless);

        controller.clearFaults();
        controller.restoreFactoryDefaults();
        controller.setIdleMode(IdleMode.kBrake);
        controller.setInverted(invert);
        controller.enableVoltageCompensation(12);
        controller.setOpenLoopRampRate(0.1);
        controller.setClosedLoopRampRate(0.1);
        controller.setSmartCurrentLimit(70, 70, 9000);

        return controller;
    }

    public void set(double left, double right) {
        left0.set(left);
        right0.set(right);
    }

    public void flip() {
        invert = !invert;
    }

    public void setMaxOutput(double maxOutput) {
        this.maxOutput = maxOutput;
    }

    @Override
    public void periodic() {
    }

}