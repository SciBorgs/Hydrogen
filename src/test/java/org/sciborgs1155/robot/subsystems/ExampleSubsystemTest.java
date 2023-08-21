package org.sciborgs1155.robot.subsystems;

import org.junit.jupiter.api.*;
import org.sciborgs1155.robot.testingUtil.BasicPackage;

public class ExampleSubsystemTest {

  ExampleSubsystem exampleSubsystem;

  @BeforeEach
  void setup() {
    BasicPackage.setupHAL();
    exampleSubsystem = new ExampleSubsystem();
  }

  @Test
  void test() {}

  @AfterEach
  void reset() throws Exception {
    BasicPackage.closeSubsystem(exampleSubsystem);
  }
}
