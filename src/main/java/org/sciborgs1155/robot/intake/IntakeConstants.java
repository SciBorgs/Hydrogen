package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.Amps;
import edu.wpi.first.units.measure.Current;

public class IntakeConstants {
    // I just copied the constants from the 2026 rebuilt intake code
    
    public static final Current CURRENT_LIMIT = Amps.of(30);

    public static final double INTAKE_POWER = 0.5;
    public static final double GEARING = 2;
}
