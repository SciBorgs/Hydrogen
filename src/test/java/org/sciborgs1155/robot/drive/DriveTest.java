package org.sciborgs1155.robot.drive;

import org.junit.jupiter.api.*;
import org.sciborgs1155.robot.testingUtil.BasicPackage;

// TODO: add comments
public class DriveTest {

  Drive drive;

  @BeforeEach
  void setup() {
    BasicPackage.setupHAL();
    drive = Drive.create();
  }

  @Test
  void conditionTest() {
    assert !drive.isFailing();
  }

  @AfterEach
  void destroy() throws Exception {
    BasicPackage.closeSubsystem(drive);
  }
}
