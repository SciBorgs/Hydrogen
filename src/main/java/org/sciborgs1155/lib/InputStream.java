package org.sciborgs1155.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

/**
 * A functional interface to aid in modifying double suppliers, such as from a joystick.
 *
 * <p>Inspired by 694's StuyLib.
 */
@FunctionalInterface
public interface InputStream extends DoubleSupplier {

  /**
   * Creates an input stream from another.
   *
   * @param base The base stream.
   * @return A new input stream.
   */
  public static InputStream of(InputStream base) {
    return base;
  }

  /**
   * Shorthand to return a double value.
   *
   * @return The value from {@link #getAsDouble()}.
   */
  public default double get() {
    return getAsDouble();
  }

  /**
   * Transforms the stream outputs by an operator.
   *
   * @param operator A function that takes in two double inputs and returns one.
   * @return A transformed stream.
   */
  public default InputStream transform(DoubleUnaryOperator operator) {
    return () -> operator.applyAsDouble(getAsDouble());
  }

  /**
   * Scales the stream outputs by a factor.
   *
   * @param factor A supplier of scaling factors.
   * @return A scaled stream.
   */
  public default InputStream scale(DoubleSupplier factor) {
    return transform(x -> x * factor.getAsDouble());
  }

  /**
   * Scales the stream outputs by a factor.
   *
   * @param factor A scaling factor.
   * @return A scaled stream.
   */
  public default InputStream scale(double factor) {
    return scale(() -> factor);
  }

  /**
   * Negates the stream outputs.
   *
   * @return A stream scaled by -1.
   */
  public default InputStream negate() {
    return scale(-1);
  }

  /**
   * Raises the stream outputs to an exponent.
   *
   * @param exponent The exponent to raise them to.
   * @return An exponentiated stream.
   */
  public default InputStream pow(double exponent) {
    return transform(x -> Math.pow(x, exponent));
  }

  /**
   * Raises the stream outputs to an exponent and keeps their original sign.
   *
   * @param exponent The exponent to raise them to.
   * @return An exponentiated stream.
   */
  public default InputStream signedPow(double exponent) {
    return transform(x -> Math.copySign(Math.pow(x, exponent), x));
  }

  /**
   * Deadbands the stream outputs by a minimum bound and scales them from 0 to a maximum bound.
   *
   * @param bound The lower bound to deadband with.
   * @param max The maximum value to scale with.
   * @return A deadbanded stream.
   */
  public default InputStream deadband(double deadband, double max) {
    return transform(x -> MathUtil.applyDeadband(x, deadband, Double.MAX_VALUE));
  }

  /**
   * Clamps the stream outputs by a maximum bound.
   *
   * @param magnitude The upper bound to clamp with.
   * @return A clamped stream.
   */
  public default InputStream clamp(double magnitude) {
    return transform(x -> MathUtil.clamp(x, -magnitude, magnitude));
  }

  /**
   * Rate limits the stream outputs by a specified rate.
   *
   * @param rate The rate in units / s.
   * @return A rate limited stream.
   */
  public default InputStream rateLimit(double rate) {
    var limiter = new SlewRateLimiter(rate);
    return transform(x -> limiter.calculate(x));
  }
}