package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.*;
import static org.sciborgs1155.robot.Intake.IntakeConstants.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.Intake.Intake;

public class IntakeTest {
  final double DELTA = 7e-1;
  static Intake intake;

  @BeforeEach
  public void setup() {
    setupHAL();
    intake = new Intake();
  }

  @Test
  public void testMovement() {
    run(intake.intake());
    fastForward(500);
    assertEquals(INTAKE_ANGULAR_SPEED, intake.motorAngularVelocity(), DELTA);
    // test forward movement

    run(intake.outtake());
    fastForward(500);
    assertEquals(-INTAKE_ANGULAR_SPEED, intake.motorAngularVelocity(), DELTA);
    // test backwards movement

    run(intake.stop());
    fastForward(500);
    assertEquals(0, intake.motorAngularVelocity(), DELTA);
    // test stop
  }
}
