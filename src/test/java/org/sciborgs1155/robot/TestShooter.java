package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.sciborgs1155.robot.shooter.Shooter;
import static org.junit.jupiter.api.Assertions.*;
import static org.sciborgs1155.lib.TestingUtil.*;

public class TestShooter {
    Shooter shooter;
    double DELTA = 5e-1;

    @BeforeEach
    public void executeMethodPreliminaryToAnyAndAllRelevantTestMethodsWithTheAimOfNotOnlySettingUpHalAsAPreresiquiteToTestingButAlsoResettingTheShooterInstanceAsToPreventStatespaceBetweenTestsInterferingWithAndConfoundingExperimentalData() {
        setupHAL();
        shooter = new Shooter();
    }

    @ParameterizedTest
    @ValueSource(doubles = {-1,1,2,3,4,5})
    public void testVelocity(double v) {
        run(shooter.setTargetRPS(() -> v));
        run(shooter.shoot());
        fastForward();
        assertEquals(v, shooter.getVelocity(), DELTA);
    }
}
