package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public final class Constants {
    public static final class DriveConstants {
        public static final int kLeftMotor0Port = 10;
        public static final int kLeftMotor1Port = 11;
        public static final int kRightMotor0Port = 20;
        public static final int kRightMotor1Port = 21;

        public static final double kEncoderCPR = 42.0;
        public static final double kGearing = 11.127;
        public static final double kWheelDiameterMeters = 0.1524;
        public static final double kEncoderDistancePerPulse = 1 / (11.127 / 0.4788);
        public static final double kEncoderPulsesPerMeter = 1/ kEncoderDistancePerPulse;

        public static final double kTrackwidthMeters = 0.676; // track width for impulse
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);

        public static final double ksVolts = 0.524; // s, v, a for impulse //ks from characterization old: 0.218
        public static final double kvVoltSecondsPerMeter = 2.8; //kv from characterization old: 2.24
        public static final double kaVoltSecondsSquaredPerMeter = 0.0462;  //ks from characterization old: 0.516

        public static final double kPDriveVel = 0.208;

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

    public static final class OIConstants {
        public static final int kLeftStickPort = 0;
        public static final int kRightStickPort = 1;
        public static final int kControllerPort = 2;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}