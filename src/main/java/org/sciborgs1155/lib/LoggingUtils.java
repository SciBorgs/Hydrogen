package org.sciborgs1155.lib;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.struct.Struct;

import java.util.Collection;

public class LoggingUtils {
  public static void log(String identifier, int value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, long value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, float value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, double value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, boolean value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, byte[] value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, int[] value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, long[] value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, float[] value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, double[] value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, boolean[] value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, String value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, String[] value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }
  
  public static void log(String identifier, Collection<String> value) {
    log(identifier, value.toArray(String[]::new));
  }

  public static <S> void log(String identifier, S value, Struct<S> struct) {
    log(identifier, value)
  }

  /**
   * Logs an array of struct-serializable objects.
   *
   * @param identifier the identifier of the data point
   * @param value the value of the data point
   * @param struct the struct to use to serialize the objects
   * @param <S> the serializable type
   */
  <S> void log(String identifier, S[] value, Struct<S> struct);

  public static void log(String identifier, Measure<?> value) {
    log(identifier, value.baseUnitMagnitude());
  }

  public static void log(String identifier, Measure<Unit> value, Unit unit) {
    log(identifier, value.in(unit));
  }

  public static void log(String identifier, Enum<?> value) {
    log(identifier, value.name());
  }
}
