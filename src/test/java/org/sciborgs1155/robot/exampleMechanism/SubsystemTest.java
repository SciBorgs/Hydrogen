package org.sciborgs1155.robot.exampleMechanism;

import org.junit.jupiter.api.*;
import org.sciborgs1155.robot.testingUtil.BasicPackage;

// TODO: add comments
public class SubsystemTest {

  Subsystem exampleSubsystem;

  @BeforeEach
  void setup() {
    BasicPackage.setupHAL();
    exampleSubsystem = Subsystem.create();
  }

  @Test
  void conditionTest() {
    assert exampleSubsystem.exampleCondition();
  }

  @AfterEach
  void reset() throws Exception {
    BasicPackage.closeSubsystem(exampleSubsystem);
  }
}
