package org.sciborgs1155.robot.subsystems;

import org.junit.jupiter.api.*;
import org.sciborgs1155.robot.exampleMechanism.Subsystem;
import org.sciborgs1155.robot.testingUtil.BasicPackage;

// TODO: add comments
public class ExampleSubsystemTest {

  Subsystem exampleSubsystem;

  @BeforeEach
  void setup() {
    BasicPackage.setupHAL();
    exampleSubsystem = Subsystem.create();
  }

  @Test
  void test() {}

  @AfterEach
  void reset() throws Exception {
    BasicPackage.closeSubsystem(exampleSubsystem);
  }
}
