package org.sciborgs1155.lib;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.studica.frc.AHRS;
import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
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

  /** A class to represent an alerts widget on NetworkTables */
  public static class AlertsNT {
    private final StringArrayPublisher errors;
    private final StringArrayPublisher warnings;
    private final StringArrayPublisher infos;

    public AlertsNT(NetworkTable base, String name) {
      NetworkTable table = base.getSubTable(name);
      table.getStringTopic(".type").publish().set("Alerts");
      errors = table.getStringArrayTopic("errors").publish();
      warnings = table.getStringArrayTopic("warnings").publish();
      infos = table.getStringArrayTopic("infos").publish();
    }

    public void set(Set<Alert> alerts) {
      errors.set(filteredStrings(alerts, AlertType.kError));
      warnings.set(filteredStrings(alerts, AlertType.kWarning));
      infos.set(filteredStrings(alerts, AlertType.kInfo));
    }
  }

  // DATA
  private static final HashMap<Optional<BooleanSupplier>, Alert> alertReporters = new HashMap<>(0);
  private static final Set<Alert> allAlerts = new HashSet<>();
  private static final Set<Alert> activeAlerts = new HashSet<>();
  private static final Set<Alert> totalAlerts = new HashSet<>();

  // NETWORK TABLES
  private static final NetworkTable base = NetworkTableInstance.getDefault().getTable("Alerts");
  private static final AlertsNT activeAlertsNT = new AlertsNT(base, "Active Alerts");
  private static final AlertsNT totalAlertsNT = new AlertsNT(base, "Total Alerts");

  /** Polls registered fallibles. This method should be called periodically. */
  public static void update() {
    alertReporters.forEach(
        (r, a) -> {
          if (r.isPresent()) {
            a.set(r.get().getAsBoolean());
            if (r.get().getAsBoolean()) {
              report(a);
            }
          }
        });

    totalAlerts.addAll(activeAlerts);

    activeAlertsNT.set(activeAlerts);
    totalAlertsNT.set(totalAlerts);

    activeAlerts.clear();
  }

  /** Clears total alerts. */
  public static void clear() {
    totalAlerts.clear();
    activeAlerts.clear();
  }

  /** Clears alerts suppliers. */
  public static void unregisterAll() {
    alertReporters.clear();
  }

  /**
   * Returns the set of all current alerts.
   *
   * @return The set of all current alerts.
   */
  public static Set<Alert> activeAlerts() {
    return activeAlerts;
  }

  /**
   * Returns the set of all total alerts.
   *
   * @return The set of all total alerts.
   */
  public static Set<Alert> totalAlerts() {
    return totalAlerts;
  }

  /**
   * Reports an alert.
   *
   * @param alert The alert to report.
   */
  public static void report(Alert alert) {
    alert.set(true);
    activeAlerts.add(alert);
    switch (alert.getType()) {
      case kError -> DriverStation.reportError(alert.getText(), false);
      case kWarning -> DriverStation.reportWarning(alert.getText(), false);
      case kInfo -> System.out.println(alert.getText());
    }
  }

  /**
   * Reports an alert.
   *
   * @param name The name of the alert.
   * @param description The description of the alert.
   * @param type The type of the alert.
   */
  public static void report(String name, String description, AlertType type) {
    allAlerts.clear();
    alertReporters.forEach((r, a) -> allAlerts.add(a));
    boolean existed = false;
    for (Alert values : allAlerts) {
      if (values.getText() == description && values.getType() == type) {
        existed = true;
        report(values);
        break;
      }
    }
    if (!existed) {
      Alert alert = new Alert(name, description, type);
      alertReporters.put(Optional.of(() -> true), alert);
      report(alert);
    }
  }

  /**
   * Registers a new alert supplier.
   *
   * @param supplier A supplier of an optional alert.
   */
  public static void register(Optional<BooleanSupplier> condition, Alert alert) {
    alertReporters.put(condition, alert);
  }

  /**
   * Registers a new alert supplier.
   *
   * @param condition Whether a failure is occuring.
   * @param description The failure's description.
   * @param type The type of failure.
   */
  public static void register(
      BooleanSupplier condition, String name, String description, AlertType type) {
    register(Optional.of(condition), new Alert(name, description, type));
  }

  /**
   * Registers fault suppliers for a CAN-based Spark motor controller.
   *
   * @param spark The Spark Max or Spark Flex to manage.
   */
  public static void register(SparkBase spark) {
    register(
        () -> spark.getFaults().other,
        SparkUtils.name(spark),
        "other strange error",
        AlertType.kError);
    register(
        () -> spark.getFaults().motorType,
        SparkUtils.name(spark),
        "motor type error",
        AlertType.kError);
    register(
        () -> spark.getFaults().sensor, SparkUtils.name(spark), "sensor error", AlertType.kError);
    register(() -> spark.getFaults().can, SparkUtils.name(spark), "CAN error", AlertType.kError);
    register(
        () -> spark.getFaults().temperature,
        SparkUtils.name(spark),
        "temperature error",
        AlertType.kError);
    register(
        () -> spark.getFaults().gateDriver,
        SparkUtils.name(spark),
        "gate driver error",
        AlertType.kError);
    register(
        () -> spark.getFaults().escEeprom,
        SparkUtils.name(spark),
        "escEeprom? error",
        AlertType.kError);
    register(
        () -> spark.getFaults().firmware,
        SparkUtils.name(spark),
        "firmware error",
        AlertType.kError);
    register(
        () -> spark.getMotorTemperature() > 100,
        SparkUtils.name(spark),
        "motor above 100°C",
        AlertType.kError);
  }

  /**
   * Registers alert suppliers for a duty cycle encoder.
   *
   * @param encoder The duty cycle encoder to manage.
   */
  public static void register(DutyCycleEncoder encoder) {
    register(
        () -> !encoder.isConnected(),
        "Duty Cycle Encoder [" + encoder.getSourceChannel() + "]",
        "disconnected",
        AlertType.kError);
  }

  /**
   * Registers alert suppliers for a NavX.
   *
   * @param ahrs The NavX to manage.
   */
  public static void register(AHRS ahrs) {
    register(() -> !ahrs.isConnected(), "NavX", "disconnected", AlertType.kError);
  }

  /**
   * Registers alert suppliers for a Redux Boron CANandGyro.
   *
   * @param canandgyro The Redux Boron CANandGyro to manage.
   */
  public static void register(Canandgyro canandgyro) {
    register(() -> !canandgyro.isConnected(), "CANandGyro", "disconnected", AlertType.kError);
    register(
        () -> canandgyro.getActiveFaults().accelerationSaturation(),
        "CANandGyro",
        "acceleration saturated",
        AlertType.kWarning);
    register(
        () -> canandgyro.getActiveFaults().angularVelocitySaturation(),
        "CANandGyro",
        "angular velocity saturated",
        AlertType.kWarning);
    register(
        () -> canandgyro.getActiveFaults().calibrating(),
        "CANandGyro",
        "calibrating",
        AlertType.kWarning);
    register(
        () -> canandgyro.getActiveFaults().canGeneralError(),
        "CANandGyro",
        "general CAN error",
        AlertType.kError);
    register(
        () -> canandgyro.getActiveFaults().canIDConflict(),
        "CANandGyro",
        "CAN ID conflict",
        AlertType.kError);
    register(
        () -> canandgyro.getActiveFaults().outOfTemperatureRange(),
        "CANandGyro",
        "temperature error",
        AlertType.kError);
    register(
        () -> canandgyro.getActiveFaults().powerCycle(),
        "CANandGyro",
        "power cycling",
        AlertType.kWarning);
  }

  /**
   * Registers fault suppliers for a power distribution hub/panel.
   *
   * @param powerDistribution The power distribution to manage.
   */
  public static void register(PowerDistribution powerDistribution) {
    var fields = PowerDistributionFaults.class.getFields();
    for (Field fault : fields) {
      register(
          Optional.of(
              () -> {
                try {
                  if (fault.getBoolean(powerDistribution.getFaults())) {
                    return fault.getBoolean(powerDistribution.getFaults());
                  }
                } catch (Exception e) {
                  return false;
                }
                return false;
              }),
          new Alert("Power Distribution", fault.getName(), AlertType.kError));
    }
    ;
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
        AlertType.kError);
  }

  /**
   * Registers fault suppliers for a talon.
   *
   * @param talon The talon to manage.
   */
  public static void register(TalonFX talon) {
    int id = talon.getDeviceID();
    BiConsumer<StatusSignal<Boolean>, String> regFault =
        (f, d) -> register((BooleanSupplier) f, "Talon ID: " + id, d, AlertType.kError);

    // TODO: Remove all the unnecessary faults.
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
  public static boolean check(SparkBase spark) {
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
  public static boolean check(SparkBase spark, REVLibError error) {
    if (error != REVLibError.kOk) {
      report(SparkUtils.name(spark), error.name(), AlertType.kError);
      return false;
    }
    return true;
  }

  /**
   * Returns an array of descriptions of all alerts that match the specified type.
   *
   * @param alerts Alerts to search through.
   * @param type The type to filter for.
   * @return An array of description strings.
   */
  private static String[] filteredStrings(Set<Alert> alerts, AlertType type) {
    return alerts.stream()
        .filter(a -> a.getType() == type)
        .map(Alert::getText)
        .toArray(String[]::new);
  }
}
