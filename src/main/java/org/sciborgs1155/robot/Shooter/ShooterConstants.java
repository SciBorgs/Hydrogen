package org.sciborgs1155.robot.Shooter;

public class ShooterConstants {
    static int deviceID = 0;
    static int ki = 0;
    static double kp = 0.03;
    static int kd = 0;
    public static final double kSVolts = 0.05;
    public static final double kShooterFreeRPS = 5300;
    public static final double kVVoltSecondsPerRotation =
        // Should have value 12V at free speed...
        12.0 / kShooterFreeRPS;
}
