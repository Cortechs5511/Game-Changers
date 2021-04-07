// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int kLeftMotor0Port = 10;
        public static final int kLeftMotor1Port = 11;
        public static final int kRightMotor0Port = 20;
        public static final int kRightMotor1Port = 21;

        public static final double kAngleP = 0.023;
        public static final double kAngleI = 0.037;
        public static final double kAngleD = 0.00115;
    }

    public static final class ShooterConstants {
        public static final int kShoot0Port = 50;
        public static final int kShoot1Port = 51;

        public static final double kShootP = 0.00025;
        public static final double kShootI = 0;
        public static final double kShootD = 0;
        public static final double kShootFF = 0.000199;
        public static final double kIdlePower = 0.45;
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

}
