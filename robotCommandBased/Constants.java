package frc.robot;

import java.lang.invoke.MethodHandles;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    static
    {
        System.out.println("Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName());
    }
    public static final int driverControllerID = 0;

    public static class Drive
    {
        public static final double autoMinimalMoveTime = 3.;
        public static final int testMotorPort = 3; // SparkMax CAN id
        public static final double VoltageCompensation = 10;
        public static final int configRetries = 5;
        public static final int statusFrame0Periodms = 10;
        public static final int statusFrame1Periodms = 20;
        public static final int statusFrame2Periodms = 20;
        public static final double DriveStraightSlowlySpeed = 0.25;
    }

    public static class Flywheel
    {
        public static final double kAutoSpinupRPM = 600.;
        public static final double kAutoTime = 2.;
        public static final double kDriverButtonFlywheelSpeed = 500.; // establish the shooter flywheel start/stop button at speed
    }

    public static class FanFSM
    {
        public static final double kOffSpeed = 0.0;
        public static final double kHighSpeed = 1.0;
        public static final double kMediumSpeed = 0.7;
        public static final double kLowSpeed = 0.3;
    }
}
