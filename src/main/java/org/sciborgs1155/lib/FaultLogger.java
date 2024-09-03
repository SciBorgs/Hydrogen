package org.sciborgs1155.lib;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.REVLibError;
import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;

/**
 * FaultLogger allows for faults to be logged and displayed.
 *
 * <pre>
 * FaultLogger.register(spark); // registers a spark, periodically checking for hardware faults
 * spark.set(0.5);
 * FaultLogger.check(spark); // checks that the previous set call did not encounter an error.
 * </pre>
 */
public final class FaultLogger {
  /** An individual fault, containing necessary information. */
  public static record Fault(String name, String description, FaultType type) {
    @Override
    public String toString() {
      return name + ": " + description;
    }
  }

  @FunctionalInterface
  public static interface FaultReporter {
    void report();
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

  /** A class to represent an alerts widget on NetworkTables */
  public static class Alerts {
    private final StringArrayPublisher errors;
    private final StringArrayPublisher warnings;
    private final StringArrayPublisher infos;

    public Alerts(NetworkTable base, String name) {
      NetworkTable table = base.getSubTable(name);
      table.getStringTopic(".type").publish().set("Alerts");
      errors = table.getStringArrayTopic("errors").publish();
      warnings = table.getStringArrayTopic("warnings").publish();
      infos = table.getStringArrayTopic("infos").publish();
    }

    public void set(Set<Fault> faults) {
      errors.set(filteredStrings(faults, FaultType.ERROR));
      warnings.set(filteredStrings(faults, FaultType.WARNING));
      infos.set(filteredStrings(faults, FaultType.INFO));
    }
  }

  // DATA
  private static final List<FaultReporter> faultReporters = new ArrayList<>();
  private static final Set<Fault> newFaults = new HashSet<>();
  private static final Set<Fault> activeFaults = new HashSet<>();
  private static final Set<Fault> totalFaults = new HashSet<>();

  // NETWORK TABLES
  private static final NetworkTable base = NetworkTableInstance.getDefault().getTable("Faults");
  private static final Alerts activeAlerts = new Alerts(base, "Active Faults");
  private static final Alerts totalAlerts = new Alerts(base, "Total Faults");

  /** Polls registered fallibles. This method should be called periodically. */
  public static void update() {
    activeFaults.clear();

    faultReporters.forEach(FaultReporter::report);
    activeFaults.addAll(newFaults);
    newFaults.clear();

    totalFaults.addAll(activeFaults);

    activeAlerts.set(activeFaults);
    totalAlerts.set(totalFaults);
  }

  /** Clears total faults. */
  public static void clear() {
    totalFaults.clear();
    activeFaults.clear();
    newFaults.clear();
  }

  /** Clears fault suppliers. */
  public static void unregisterAll() {
    faultReporters.clear();
  }

  /**
   * Returns the set of all current faults.
   *
   * @return The set of all current faults.
   */
  public static Set<Fault> activeFaults() {
    return activeFaults;
  }

  /**
   * Returns the set of all total faults.
   *
   * @return The set of all total faults.
   */
  public static Set<Fault> totalFaults() {
    return totalFaults;
  }

  /**
   * Reports a fault.
   *
   * @param fault The fault to report.
   */
  public static void report(Fault fault) {
    newFaults.add(fault);
    switch (fault.type) {
      case ERROR -> DriverStation.reportError(fault.toString(), false);
      case WARNING -> DriverStation.reportWarning(fault.toString(), false);
      case INFO -> System.out.println(fault.toString());
    }
  }

  /**
   * Reports a fault.
   *
   * @param name The name of the fault.
   * @param description The description of the fault.
   * @param type The type of the fault.
   */
  public static void report(String name, String description, FaultType type) {
    report(new Fault(name, description, type));
  }

  /**
   * Registers a new fault supplier.
   *
   * @param supplier A supplier of an optional fault.
   */
  public static void register(Supplier<Optional<Fault>> supplier) {
    faultReporters.add(() -> supplier.get().ifPresent(FaultLogger::report));
  }

  /**
   * Registers a new fault supplier.
   *
   * @param condition Whether a failure is occuring.
   * @param description The failure's description.
   * @param type The type of failure.
   */
  public static void register(
      BooleanSupplier condition, String name, String description, FaultType type) {
    faultReporters.add(
        () -> {
          if (condition.getAsBoolean()) {
            report(name, description, type);
          }
        });
  }

  /**
   * Registers fault suppliers for a CAN-based Spark motor controller.
   *
   * @param spark The Spark Max or Spark Flex to manage.
   */
  public static void register(CANSparkBase spark) {
    faultReporters.add(
        () -> {
          for (FaultID fault : FaultID.values()) {
            if (spark.getFault(fault)) {
              report(SparkUtils.name(spark), fault.name(), FaultType.ERROR);
            }
          }
        });
    register(
        () -> spark.getMotorTemperature() > 100,
        SparkUtils.name(spark),
        "motor above 100°C",
        FaultType.WARNING);
  }

  /**
   * Registers fault suppliers for a duty cycle encoder.
   *
   * @param encoder The duty cycle encoder to manage.
   */
  public static void register(DutyCycleEncoder encoder) {
    register(
        () -> !encoder.isConnected(),
        "Duty Cycle Encoder [" + encoder.getSourceChannel() + "]",
        "disconnected",
        FaultType.ERROR);
  }

  /**
   * Registers fault suppliers for a NavX.
   *
   * @param ahrs The NavX to manage.
   */
  public static void register(AHRS ahrs) {
    register(() -> !ahrs.isConnected(), "NavX", "disconnected", FaultType.ERROR);
  }

  /**
   * Registers fault suppliers for a power distribution hub/panel.
   *
   * @param powerDistribution The power distribution to manage.
   */
  public static void register(PowerDistribution powerDistribution) {
    var fields = PowerDistributionFaults.class.getFields();
    faultReporters.add(
        () -> {
          try {
            PowerDistributionFaults faults = powerDistribution.getFaults();
            for (Field fault : fields) {
              if (fault.getBoolean(faults)) {
                report("Power Distribution", fault.getName(), FaultType.ERROR);
              }
            }
          } catch (Exception e) {
          }
        });
  }

  /**
   * Registers fault suppliers for a camera.
   *
   * @param camera The camera to manage.
   */
  public static void register(PhotonCamera camera) {
    register(
        () -> !camera.isConnected(),
        "Photon Camera [" + camera.getName() + "]",
        "disconnected",
        FaultType.ERROR);
  }

  /**
   * Registers fault suppliers for a talon.
   *
   * @param talon The talon to manage.
   */
  public static void register(TalonFX talon) {
    int id = talon.getDeviceID();
    BiConsumer<StatusSignal<Boolean>, String> regFault =
        (f, d) -> register((BooleanSupplier) f, "Talon ID: " + id, d, FaultType.ERROR);

    // TODO: Remove all the unnecessary faults
    regFault.accept(talon.getFault_Hardware(), "Hardware fault occurred");
    regFault.accept(talon.getFault_ProcTemp(), "Processor temperature exceeded limit");
    regFault.accept(talon.getFault_Hardware(), "Hardware fault occurred");
    regFault.accept(talon.getFault_ProcTemp(), "Processor temperature exceeded limit");
    regFault.accept(talon.getFault_DeviceTemp(), "Device temperature exceeded limit");
    regFault.accept(
        talon.getFault_Undervoltage(), "Device supply voltage dropped to near brownout levels");
    regFault.accept(
        talon.getFault_BootDuringEnable(), "Device boot while detecting the enable signal");
    regFault.accept(
        talon.getFault_UnlicensedFeatureInUse(),
        "An unlicensed feature is in use, device may not behave as expected.");
    regFault.accept(
        talon.getFault_BridgeBrownout(),
        "Bridge was disabled most likely due to supply voltage dropping too low.");
    regFault.accept(talon.getFault_RemoteSensorReset(), "The remote sensor has reset.");
    regFault.accept(
        talon.getFault_MissingDifferentialFX(),
        "The remote Talon FX used for differential control is not present on CAN Bus.");
    regFault.accept(
        talon.getFault_RemoteSensorPosOverflow(), "The remote sensor position has overflowed.");
    regFault.accept(
        talon.getFault_OverSupplyV(),
        "Supply Voltage has exceeded the maximum voltage rating of device.");
    regFault.accept(talon.getFault_UnstableSupplyV(), "Supply Voltage is unstable.");
    regFault.accept(
        talon.getFault_ReverseHardLimit(),
        "Reverse limit switch has been asserted.  Output is set to neutral.");
    regFault.accept(
        talon.getFault_ForwardHardLimit(),
        "Forward limit switch has been asserted.  Output is set to neutral.");
    regFault.accept(
        talon.getFault_ReverseSoftLimit(),
        "Reverse soft limit has been asserted.  Output is set to neutral.");
    regFault.accept(
        talon.getFault_ForwardSoftLimit(),
        "Forward soft limit has been asserted.  Output is set to neutral.");
    regFault.accept(
        talon.getFault_RemoteSensorDataInvalid(), "The remote sensor's data is no longer trusted.");
    regFault.accept(
        talon.getFault_FusedSensorOutOfSync(),
        "The remote sensor used for fusion has fallen out of sync to the local sensor.");
    regFault.accept(talon.getFault_StatorCurrLimit(), "Stator current limit occured.");
    regFault.accept(talon.getFault_SupplyCurrLimit(), "Supply current limit occured.");
    regFault.accept(
        talon.getFault_UsingFusedCANcoderWhileUnlicensed(),
        "Using Fused CANcoder feature while unlicensed. Device has fallen back to remote CANcoder.");
  }

  /**
   * Reports REVLibErrors from a spark.
   *
   * <p>This should be called immediately after any call to the spark.
   *
   * @param spark The spark to report REVLibErrors from.
   * @return If the spark is working without errors.
   */
  public static boolean check(CANSparkBase spark) {
    REVLibError error = spark.getLastError();
    return check(spark, error);
  }

  /**
   * Reports REVLibErrors from a spark.
   *
   * <p>This should be called immediately after any call to the spark.
   *
   * @param spark The spark to report REVLibErrors from.
   * @param error Any REVLibErrors that may be returned from a method for a spark.
   * @return If the spark is working without errors.
   */
  public static boolean check(CANSparkBase spark, REVLibError error) {
    if (error != REVLibError.kOk) {
      report(SparkUtils.name(spark), error.name(), FaultType.ERROR);
      return false;
    }
    return true;
  }

  /**
   * Returns an array of descriptions of all faults that match the specified type.
   *
   * @param type The type to filter for.
   * @return An array of description strings.
   */
  private static String[] filteredStrings(Set<Fault> faults, FaultType type) {
    return faults.stream()
        .filter(a -> a.type() == type)
        .map(Fault::toString)
        .toArray(String[]::new);
  }
}
