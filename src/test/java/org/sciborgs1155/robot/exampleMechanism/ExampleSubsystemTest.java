package org.sciborgs1155.robot.exampleMechanism;

import org.junit.jupiter.api.*;
import org.sciborgs1155.robot.testingUtil.BasicPackage;

// TODO: add comments
public class ExampleSubsystemTest {

  ExampleSubsystem exampleSubsystem;

  @BeforeEach
  void setup() {
    BasicPackage.setupHAL();
    exampleSubsystem = ExampleSubsystem.create();
  }

  @Test
  void conditionTest() {
    assert exampleSubsystem.exampleCondition();
  }

  @AfterEach
  void destroy() throws Exception {
    BasicPackage.closeSubsystem(exampleSubsystem);
  }
}
