package org.sciborgs1155.robot;

/**
 * Constants is a globally accessible class for storing immutable values. Every value should be
 * <code>public static final</code>.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *
 * <p><b>Units</b>
 *
 * <ul>
 *   <li>length: meters
 *   <li>time: seconds
 *   <li>angle: radians
 * </ul>
 *
 * @see MotorConfig
 * @see Conversion
 * @see PIDConstants
 * @see Constraints
 */
public final class Constants {
  public static final double PERIOD = 0.02;

  public static class OI {
    public static final int DRIVER = 1;
    public static final int OPERATOR = 0;
  }
}
