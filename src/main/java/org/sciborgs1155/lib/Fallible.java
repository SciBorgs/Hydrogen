package org.sciborgs1155.lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.REVLibError;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;

@FunctionalInterface
public interface Fallible extends Sendable {

  public static record Fault(String description, FaultType type, double timestamp) {
    public Fault(String description, FaultType type) {
      this(description, type, Timer.getFPGATimestamp());
    }
  }

  public static enum FaultType {
    INFO,
    WARNING,
    ERROR,
  }

  public default List<Fault> from(String label, FaultType type, boolean condition) {
    return condition ? List.of(new Fault(label, type)) : List.of();
  }

  public default List<Fault> from(CANSparkMax sparkMax) {
    List<Fault> faults = new ArrayList<>();
    REVLibError err = sparkMax.getLastError();
    int id = sparkMax.getDeviceId();
    if (err != REVLibError.kOk) {
      faults.add(
          new Fault(String.format("SparkMax [%d]: Error: %s", id, err.name()), FaultType.ERROR));
    }
    for (FaultID fault : FaultID.values()) {
      if (sparkMax.getFault(fault)) {
        faults.add(
            new Fault(
                String.format("SparkMax [%d]: Fault: %s", id, fault.name()), FaultType.WARNING));
      }
    }
    return faults;
  }

  public default List<Fault> from(DutyCycleEncoder encoder) {
    return from(
        String.format("DutyCycleEncoder [%d]: Disconnected", encoder.getSourceChannel()),
        FaultType.ERROR,
        !encoder.isConnected());
  }

  public default List<Fault> from(Fault... faults) {
    return from(List.of(faults));
  }

  @SafeVarargs
  public static List<Fault> from(List<Fault>... faults) {
    // calculate length to be allocated
    int len = 0;
    for (List<Fault> f : faults) {
      len += f.size();
    }

    List<Fault> allFaults = new ArrayList<>(len);

    for (List<Fault> f : faults) {
      allFaults.addAll(f);
    }

    return allFaults;
  }

  public List<Fault> getFaults();

  public default boolean isFailing() {
    for (Fault fault : getFaults()) {
      if (fault.type() == FaultType.ERROR) {
        return true;
      }
    }
    return false;
  }

  public default Trigger getTrigger() {
    return new Trigger(this::isFailing);
  }

  public default void onFailing(Command command) {
    getTrigger().debounce(0.1).onTrue(command);
  }

  public default String[] getStrings(FaultType type) {
    return getFaults().stream()
        .filter(a -> a.type() == type)
        .map(Fault::description)
        .toArray(String[]::new);
  }

  @Override
  public default void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Alerts");
    builder.addStringArrayProperty("errors", () -> getStrings(FaultType.ERROR), null);
    builder.addStringArrayProperty("warnings", () -> getStrings(FaultType.WARNING), null);
    builder.addStringArrayProperty("infos", () -> getStrings(FaultType.INFO), null);
  }
}
