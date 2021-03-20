package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

import java.util.function.Supplier;

public class Feeder extends SubsystemBase {
    private final WPI_VictorSPX feeder0 = new WPI_VictorSPX(FeederConstants.kFeeder0Port); // feeder 0 and 1 are tower
    private final WPI_VictorSPX feeder1 = new WPI_VictorSPX(FeederConstants.kFeeder1Port);
    private final WPI_VictorSPX feeder2 = new WPI_VictorSPX(FeederConstants.kFeeder2Port); // feeder 2, 3 are feeder wheels
    private final WPI_VictorSPX feeder3 = new WPI_VictorSPX(FeederConstants.kFeeder3Port);

    private final DigitalInput bottomSensor = new DigitalInput(FeederConstants.kBottomSensorPort);
    private final DigitalInput topSensor = new DigitalInput(FeederConstants.kTopSensorPort);
    private final DigitalInput greenSensor = new DigitalInput(FeederConstants.kGreenSensorPort);
    private final DigitalInput blackSensor = new DigitalInput(FeederConstants.kBlackSensorPort);

    // private final Encoder intakeEncoder = new Encoder(4, 5); // wrist -- inop
    private final Encoder feedEncoder = new Encoder(6, 7); // tower -- inop

    public Supplier<Boolean> getBottomSensor = bottomSensor::get;
    public Supplier<Boolean> getTopSensor = topSensor::get;
    public Supplier<Boolean> getGreenSensor = greenSensor::get;
    public Supplier<Boolean> getBlackSensor = blackSensor::get;

    public Feeder() {
        feeder0.configFactoryDefault();
        feeder1.configFactoryDefault();
        feeder2.configFactoryDefault();
        feeder2.configFactoryDefault();

        feeder0.setNeutralMode(NeutralMode.Brake);
        feeder1.setNeutralMode(NeutralMode.Brake);
        feeder2.setNeutralMode(NeutralMode.Brake);
        feeder2.setNeutralMode(NeutralMode.Brake);

        feeder0.setInverted(false);
        feeder1.setInverted(true);
        feeder2.setInverted(true);
        feeder3.setInverted(true);
    }

    @Override
    public void periodic() {
    }

    public void setFeederSpeed(double input) {
        feeder0.set(input);
        feeder1.set(input);
    }

    public void setFeeder2Speed(double input) {
        feeder2.set(input);
    }

    public void setFeeder3Speed(double input) {
        feeder3.set(input);
    }

    public void resetFeedEncoder() {
        feedEncoder.reset();
    }
}