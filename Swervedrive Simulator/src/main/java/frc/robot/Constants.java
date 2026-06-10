package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {

    public static class DriveConstants {
            public static final double driveP = 0;
            public static final double driveI = 0;
            public static final double driveD = 0;

            public static final double turnP = 0;
            public static final double turnI = 0;
            public static final double turnD = 0;

            public static final double momentOfInertia = 0;
            public static final double driveGearRatio = 6.12;
            public static final double turnGearRatio = 150.0 / 7;
            public static final double wheelRadiusMeters = 10;

            public static final int[] turnCANIDs = { 1, 2, 3, 4 };
            public static final int[] driveCANIDs = { 5, 6, 7, 8 };
            public static final int[] canCoderCANIDs = { 9, 10, 11, 12 };
            public static final int pigeonCANID = 13;

            public static final double kWheelDistanceMetersX = Units.inchesToMeters(25 - 5.25); // forward/back
            public static final double kWheelDistanceMetersY = Units.inchesToMeters(29 - 5.25); // left/right

            public static final double kSlowedMult = 0.12;
            
            public static final double kMaxWheelSpeed = 20; // m/s?
            public static final double kMagVelLimit = 5.7; // m/s
            public static final double kRotVelLimit = 2 * (2 * Math.PI); // rad/s

            public static final double crossbuckDelaySeconds = .3;


    }
}
