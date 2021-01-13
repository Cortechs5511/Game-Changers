package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor0Port = 20;
    public static final int kLeftMotor1Port = 21;
    public static final int kRightMotor0Port = 10;
    public static final int kRightMotor1Port = 11;

    public static final int kEncoderCPR = 42;
    public static final double kWheelDiameterMeters = 0.1524;
    public static final double kEncoderDistancePerPulse = 2.54 / 43.09861; 
    public static final double kEncoderPulsesPerMeter = 43.09861 / 2.54;

    public static final double kTrackwidthMeters = 0.73369; // track width for impulse
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);

    // public static final double enc_ToIn = 137.25 / (((5482.95 - 2949.08) + (4903.02 - 2375.024658)) / 2);
    // public static final double encToM = enc_ToIn * .0254;

    public static final double ksVolts = 0.591; // s, v, a for impulse
    public static final double kvVoltSecondsPerMeter = 0.00299;
    public static final double kaVoltSecondsSquaredPerMeter = 0.000438;

    public static final double kPDriveVel = 0.0293;

    public static final double kLeftP = 0.0294; // 0.146;
    public static final double kLeftI = 0;
    public static final double kLeftD = 14.6;
    public static final double kLeftFF = 0;

    public static final double kRightP = 0.0292; // 0.152;
    public static final double kRightI = 0;
    public static final double kRightD = 14.5;
    public static final double kRightFF = 0;

    public static final double kAngleP = 0.03; // not implemented yet pending final testing
    public static final double kAngleI = 0.02;
    public static final double kAngleD = 0.002;
  }

  public static final class FeederConstants {
    public static final int kWristPort = 30;
    public static final int kIntakePort = 31;

    public static final int kFeeder0Port = 40;
    public static final int kFeeder1Port = 41;
    public static final int kFeeder2Port = 42;
    public static final int kFeeder3Port = 43;

    public static final int kTopSensorPort = 0;
    public static final int kBottomSensorPort = 1;
    public static final int kGreenSensorPort = 2;
    public static final int kBlackSensorPort = 3;
  }

  public static final class ShooterConstants {
    public static final int kShoot0Port = 50;
    public static final int kShoot1Port = 51;

    public static final double kShootP = 0.000275; // not implemented yet pending final testing
    public static final double kShootI = 0;
    public static final double kShootD = 0.008;
    public static final double kShootFF = 0.000202;

  }

  public static final class ClimberConstants {
    public static final int kClimb0Port = 60;
    public static final int kClimb1Port = 61;
  }

  public static final class OIConstants {
    public static final int kLeftStickPort = 0;
    public static final int kRightStickPort = 1;
    public static final int kControllerPort = 2;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}