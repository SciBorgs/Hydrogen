package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.*;

import java.util.stream.Stream;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.sciborgs1155.robot.Drive.Drive;

public class DriveTest {
  static final double DELTA = 2;

  Drive drive;

  @BeforeEach
  public void setup() {
    drive = new Drive();
    setupHAL();
  }

  @ParameterizedTest
  @MethodSource("genTestSpeeds")
  public void atSetpoint(double lSpeed, double rSpeed) {
    run(drive.setMotorSpeeds(() -> lSpeed, () -> rSpeed));
    fastForward();
    assertEquals(lSpeed, drive.getLeftSpeed(), DELTA);
    assertEquals(rSpeed, drive.getRightSpeed(), DELTA);
  }

  public static Stream<Arguments> genTestSpeeds() {
    return Stream.of(
        Arguments.of(1, 1),
        Arguments.of(2, 2),
        Arguments.of(-3, -3),
        Arguments.of(4, -4),
        Arguments.of(5, 3),
        Arguments.of(-4, 3.2),
        Arguments.of(5, 2),
        Arguments.of(5, 2));
  }
}
