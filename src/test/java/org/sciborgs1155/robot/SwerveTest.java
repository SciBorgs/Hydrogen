package org.sciborgs1155.robot;

import static org.sciborgs1155.lib.TestingUtil.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.drive.Drive;

/** Swerve test. Currently incomplete and does nothing. */
public class SwerveTest {
  Drive drive;

  @BeforeEach
  public void setup() {
    setupHAL();
    drive = Drive.create();
  }

  @Test
  public void reachSetpoint() {
    run(drive.drive(() -> 1, () -> 1, () -> 1));
    fastForward();
  }
}
