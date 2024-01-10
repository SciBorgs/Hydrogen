package org.sciborgs1155.robot.shooter;

public class ShooterConstants {
  static int deviceID = 0;
  static double ki = 0.0; // i heard advice to keep the PID I at 0
  static double kp = 1.1;
  static double kd = 0; // i tried
  public static final double kSVolts = 0.05;
  public static final double kShooterFreeRPS = 5300;
  public static final double kVVoltSecondsPerRotation =
      // Should have value 12V at free speed...
      12.0 / kShooterFreeRPS;
}
