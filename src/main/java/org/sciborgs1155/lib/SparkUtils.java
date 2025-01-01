package org.sciborgs1155.lib;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SignalsConfig;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;

/** Utility class for configuration of Spark motor controllers */
public class SparkUtils {
  private static final List<Runnable> runnables = new ArrayList<>();

  public static void addChecker(Runnable runnable) {
    runnables.add(runnable);
  }

  public static List<Runnable> getRunnables() {
    return runnables;
  }

  // REV's docs have the size of a signed value of 65535ms for the max period
  // https://docs.revrobotics.com/brushless/spark-max/control-interfaces#periodic-status-frames
  // The actual max is half of this (32767ms)
  // https://www.chiefdelphi.com/t/revlibs-documented-maximum-status-frame-period-limit-is-wrong/458845, https://www.chiefdelphi.com/t/extreme-can-utilization-but-parameters-set-ok/456613/6
  public static final int FRAME_STRATEGY_DISABLED = 32767;
  public static final int FRAME_STRATEGY_SLOW = 400;
  public static final int FRAME_STRATEGY_MEDIUM = 100;
  public static final int FRAME_STRATEGY_FAST = 20;
  public static final int FRAME_STRATEGY_VERY_FAST = 10;

  public static final int THROUGHBORE_CPR = 8192;

  public static final int MAX_ATTEMPTS = 3;

  /**
   * Formats the name of a spark with its CAN ID.
   *
   * @param spark The spark to find the name of.
   * @return The name of a spark.
   */
  public static String name(SparkBase spark) {
    return "Spark [" + spark.getDeviceId() + "]";
  }

  /** Represents a type of sensor that can be plugged into the spark */
  public static enum Sensor {
    INTEGRATED,
    ANALOG,
    ALTERNATE,
    ABSOLUTE;
  }

  /** Represents a type of data that can be sent from the spark */
  public static enum Data {
    POSITION,
    VELOCITY,
    CURRENT,
    TEMPERATURE,
    INPUT_VOLTAGE,
    APPLIED_OUTPUT;
  }

  /**
   * Creates a SignalsConfig for a configuring a spark to send only specified data at high rates.
   *
   * @param data The data that the spark needs to send to the RIO.
   * @param sensors The sensors that provide data for the spark needs to send to the RIO.
   * @param withFollower Whether this spark has a following motor via {@link
   *     SparkBase#follow(SparkBase)}.
   * @see Sensor
   * @see Data
   * @see https://docs.revrobotics.com/brushless/spark-max/control-interfaces
   */
  public static SignalsConfig getSignalsConfigurationFrameStrategy(
      Set<Data> data, Set<Sensor> sensors, boolean withFollower) {
    int status0 = FRAME_STRATEGY_MEDIUM; // output, bus voltage, temperature, limits | default 10
    int status1 = FRAME_STRATEGY_MEDIUM; // faults, warnings | default 20
    int status2 = FRAME_STRATEGY_SLOW; // integrated velocity, position| default 20
    int status3 = FRAME_STRATEGY_DISABLED; // analog encoder | default 20
    int status4 = FRAME_STRATEGY_DISABLED; // external or alternate encoder | default 20
    int status5 = FRAME_STRATEGY_DISABLED; // absolute encoder | default 20
    // int status6 = FRAME_STRATEGY_DISABLED;
    // nonexistent? literally has no config method modifying it...
    int status7 = FRAME_STRATEGY_DISABLED; // IAccum
    // // status frame 7 is cursed, the only mention i found of it in rev's docs is at
    // // this page doesnt exist anymore!
    // https://docs.revrobotics.com/brushless/spark-flex/revlib/spark-flex-firmware-changelog#breaking-changes
    // // if it's only IAccum, there's literally no reason to enable the frame

    if (withFollower || data.contains(Data.APPLIED_OUTPUT) || data.contains(Data.TEMPERATURE)) {
      status0 = FRAME_STRATEGY_VERY_FAST;
    }

    if (sensors.contains(Sensor.INTEGRATED) && data.contains(Data.VELOCITY)
        || sensors.contains(Sensor.INTEGRATED) && data.contains(Data.POSITION)
        || data.contains(Data.INPUT_VOLTAGE)
        || data.contains(Data.CURRENT)) {
      status2 = FRAME_STRATEGY_FAST;
    }

    if (sensors.contains(Sensor.ANALOG)
        && (data.contains(Data.VELOCITY) || data.contains(Data.POSITION))) {
      status3 = FRAME_STRATEGY_FAST;
    }

    if (sensors.contains(Sensor.ALTERNATE)
        && (data.contains(Data.VELOCITY) || data.contains(Data.POSITION))) {
      status4 = FRAME_STRATEGY_FAST;
    }

    if (sensors.contains(Sensor.ABSOLUTE)) {
      if (data.contains(Data.POSITION)) {
        status5 = FRAME_STRATEGY_FAST;
      }
    }

    SignalsConfig config = new SignalsConfig();
    config =
        config
            .appliedOutputPeriodMs(status0)
            .faultsPeriodMs(status1)
            .primaryEncoderPositionPeriodMs(status2)
            .analogVoltagePeriodMs(status3)
            .externalOrAltEncoderPosition(status4)
            .absoluteEncoderPositionPeriodMs(status5)
            .iAccumulationPeriodMs(status7);

    return config;
  }

  /**
   * Configures a SignalsConfig that sends nothing except output and faults. This means most data
   * will not be accessible.
   */
  public static SignalsConfig getStatusConfigurationOfNothingFrameStrategy() {
    return getSignalsConfigurationFrameStrategy(Set.of(), Set.of(), false);
  }

  /**
   * Wraps the value of a call into an optional depending on the spark's indicated last error.
   *
   * @param <T> The type of value.
   * @param spark The spark to check for errors.
   * @param value The value to wrap.
   * @return An optional that may contain the value.
   */
  public static <T> Optional<T> wrapCall(SparkBase spark, T value) {
    if (FaultLogger.check(spark)) {
      return Optional.of(value);
    }
    return Optional.empty();
  }
}
