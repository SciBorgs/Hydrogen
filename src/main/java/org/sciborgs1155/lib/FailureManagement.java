package org.sciborgs1155.lib;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.REVLibError;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;

public final class FailureManagement implements Sendable {
  private static FailureManagement instance;

  /**
   * Returns the singleton instance of failure management.
   *
   * @return The instance of failure management.
   */
  public static FailureManagement getInstance() {
    if (instance == null) {
      instance = new FailureManagement();
    }
    return instance;
  }

  private FailureManagement() {}

  /** An individual fault, containing necessary information. */
  public static record Fault(String description, FaultType type, double timestamp) {
    public Fault(String description, FaultType type) {
      this(description, type, Timer.getFPGATimestamp());
    }

    @Override
    public int hashCode() {
      return Objects.hash(description, type);
    }
  }

  /**
   * The type of fault, used for detecting whether the fallible is in a failure state and displaying
   * to NetworkTables.
   */
  public static enum FaultType {
    INFO,
    WARNING,
    ERROR,
  }

  private final List<Supplier<Optional<Fault>>> faultSuppliers = new ArrayList<>();
  private final Set<Fault> faults = new HashSet<>();

  /** Polls registered fallibles. This method should be called periodically. */
  public void run() {
    faultSuppliers.stream()
        .map(s -> s.get())
        .flatMap(Optional::stream)
        .collect(Collectors.toCollection(() -> faults));
  }

  /**
   * Returns a list of all current faults.
   *
   * @return A list of all current faults.
   */
  public Set<Fault> getFaults() {
    return faults;
  }

  /**
   * Returns an array of descriptions of all faults that match the specified type.
   *
   * @param type The type to filter for.
   * @return An array of description strings.
   */
  public String[] getStrings(FaultType type) {
    return getFaults().stream()
        .filter(a -> a.type() == type)
        .map(Fault::description)
        .toArray(String[]::new);
  }

  /**
   * Registers a new fallible supplier.
   *
   * @param supplier A supplier of an optional fault.
   */
  public void register(Supplier<Optional<Fault>> supplier) {
    faultSuppliers.add(supplier);
  }

  /**
   * Registers a new fallible supplier.
   *
   * @param condition Whether a failure is occuring.
   * @param fault The failure that is occuring.
   */
  public void register(BooleanSupplier condition, Supplier<Fault> fault) {
    faultSuppliers.add(
        () -> condition.getAsBoolean() ? Optional.of(fault.get()) : Optional.empty());
  }

  /**
   * Registers fallible suppliers for a CAN-based Spark motor controller.
   *
   * @param spark The Spark Max or Spark Flex to manage.
   */
  public void register(CANSparkBase spark) {
    int id = spark.getDeviceId();

    register(
        () -> {
          REVLibError err = spark.getLastError();
          return err == REVLibError.kOk
              ? Optional.empty()
              : Optional.of(
                  new Fault(
                      String.format("Spark [%d]: Error: %s", id, err.name()), FaultType.ERROR));
        });

    for (FaultID fault : FaultID.values()) {
      register(
          () -> spark.getFault(fault),
          () ->
              new Fault(
                  String.format("Spark [%d]: Fault: %s", id, fault.name()), FaultType.WARNING));
    }
  }

  /**
   * Registers fallible suppliers for a duty cycle encoder.
   *
   * @param encoder The duty cycle encoder to manage.
   */
  public void register(DutyCycleEncoder encoder) {
    register(
        () -> !encoder.isConnected(),
        () -> new Fault(String.format("DutyCycleEncoder [%d]: Disconnected"), FaultType.ERROR));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Alerts");
    builder.addStringArrayProperty("errors", () -> getStrings(FaultType.ERROR), null);
    builder.addStringArrayProperty("warnings", () -> getStrings(FaultType.WARNING), null);
    builder.addStringArrayProperty("infos", () -> getStrings(FaultType.INFO), null);
  }
}
