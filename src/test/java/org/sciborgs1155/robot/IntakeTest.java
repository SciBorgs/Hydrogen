package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.*;
import static org.sciborgs1155.robot.Intake.IntakeConstants.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.Intake.Intake;

public class IntakeTest {
  final double DELTA = 2;
  static Intake intake;

  @BeforeEach
  public void setup() {
    setupHAL();
    intake = new Intake();
  }

  @Test
  public void testMovementForward() {
    run(intake.intake());
    assertEquals(INTAKE_SPEED, intake.targetSpeed);
  }

  @Test
  public void testMovementBack() {
    run(intake.outtake());
    assertEquals(INTAKE_SPEED * -1, intake.targetSpeed);
  }

  @Test
  public void reachesSetpointWhenMovingForward() {
    run(intake.intake());
    fastForward(500);
    assertEquals(INTAKE_SPEED, intake.speed(), DELTA);
  }

  @Test
  public void reachesSetpointWhenMovingBack() {
    run(intake.outtake());
    fastForward(500);
    assertEquals(-INTAKE_SPEED, intake.speed(), DELTA);
  }
}
