package frc.robot.util;

/**
 * A class that holds important values that are easily accessable.
 */
public class NumberConstants {
    // region Controller Constants
    public static final double driveKp = 0;

    public static final double driveKs = 0;
    public static final double driveKv = 0;
    public static final double driveKa = 0;

    public static final double Ram_b = 2.0;
    public static final double Ram_zeta = 0.7;
    // endregion

    // region Robot Speeds
    public static final double DRIVE_MAX_POWER = 1;
    public static final double DRIVE_TELEOP_MAX_POWER = 0.5;
    public static final double DRIVE_BOOST_POWER = 0.95;

    public static final double kMaxSpeedMetersPerSecond = 0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0;
    // endregion

    // region Encoder Values
    public static final double NEO_ENCODER_TPR = 42;
    public static final double DRIVE_GEAR_RATIO = 4.225;
    public static final double WHEELDIAMETER = 6;
    // endregion

    // region Measruements
    // vision
    public static final double LIME_HEIGHT = 0;
    public static final double LIME_ANGLE = 0;

    // robot
    public static final double ROBOT_WIDTH = 25;
    // endregion

}
