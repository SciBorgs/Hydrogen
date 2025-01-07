package org.sciborgs1155.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class FaultLoggerTest {

  @BeforeAll
  public static void reset() {
    FaultLogger.clear();
    FaultLogger.unregisterAll();
  }

  @BeforeEach
  public void setup() {
    setupTests();
    FaultLogger.clear();
  }

  @Test
  void report() {
    NetworkTable base = NetworkTableInstance.getDefault().getTable("Alerts");
    var activeInfos =
        base.getSubTable("Active Alerts").getStringArrayTopic("infos").subscribe(new String[10]);
    var totalErrors =
        base.getSubTable("Total Alerts").getStringArrayTopic("errors").subscribe(new String[10]);
    FaultLogger.update();
    FaultLogger.report("Test", "Example", AlertType.kInfo);
    assertEquals(1, FaultLogger.activeAlerts().size());
    FaultLogger.update();
    assertEquals(1, FaultLogger.totalAlerts().size());
    assertEquals(1, activeInfos.get().length);
    assertEquals(0, totalErrors.get().length);

    // duplicate
    FaultLogger.report("Test", "Example", AlertType.kInfo);
    assertEquals(1, FaultLogger.activeAlerts().size());
    FaultLogger.update();
    assertEquals(1, FaultLogger.totalAlerts().size());
    assertEquals(1, activeInfos.get().length);
    assertEquals(0, totalErrors.get().length);

    FaultLogger.report("Test2", "Example2", AlertType.kError);
    assertEquals(1, FaultLogger.activeAlerts().size());
    FaultLogger.update();
    assertEquals(2, FaultLogger.totalAlerts().size());
    assertEquals(0, activeInfos.get().length);
    assertEquals(1, totalErrors.get().length);
  }

  @Test
  void register() {
    NetworkTable base = NetworkTableInstance.getDefault().getTable("Alerts");
    var activeErrors =
        base.getSubTable("Active Alerts").getStringArrayTopic("errors").subscribe(new String[10]);
    var totalErrors =
        base.getSubTable("Total Alerts").getStringArrayTopic("errors").subscribe(new String[10]);

    FaultLogger.update();
    FaultLogger.register(() -> true, "Recurring Test", "Idk", AlertType.kError);
    FaultLogger.update();
    FaultLogger.update();

    // System.out.println(totalErrors.get().toString());
    for (var e : totalErrors.get()) {
      System.out.println(e);
    }

    assertEquals(1, activeErrors.get().length);
    assertEquals(1, totalErrors.get().length);
  }
}
