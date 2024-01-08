package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Velocity;
import java.util.List;

public final class DriveConstants {
  public static final Measure<Velocity<Distance>> MAX_SPEED = MetersPerSecond.of(4.8);
  public static final Measure<Velocity<Angle>> MAX_ANGULAR_SPEED = RadiansPerSecond.of(2 * Math.PI);
  public static final Measure<Velocity<Velocity<Distance>>> MAX_ACCEL =
      MetersPerSecondPerSecond.of(6.5);
  public static final Measure<Velocity<Velocity<Angle>>> MAX_ANGULAR_ACCEL =
      RadiansPerSecond.per(Second).of(2 * Math.PI);

  public static final Measure<Distance> TRACK_WIDTH = Meters.of(0.5715);
  // Distance between centers of right and left wheels on robot
  public static final Measure<Distance> WHEEL_BASE = Meters.of(0.5715);
  // Distance between front and back wheels on robot

  public static final Translation2d[] MODULE_OFFSET = {
    new Translation2d(WHEEL_BASE.in(Meters) / 2, TRACK_WIDTH.in(Meters) / 2), // front left
    new Translation2d(WHEEL_BASE.in(Meters) / 2, -TRACK_WIDTH.in(Meters) / 2), // front right
    new Translation2d(-WHEEL_BASE.in(Meters) / 2, TRACK_WIDTH.in(Meters) / 2), // rear left
    new Translation2d(-WHEEL_BASE.in(Meters) / 2, -TRACK_WIDTH.in(Meters) / 2) // rear right
  };

  // angular offsets of the modules, since we use absolute encoders
  // ignored (used as 0) in simulation because the simulated robot doesn't have offsets
  public static final List<Measure<Angle>> ANGULAR_OFFSETS =
      List.of(
          Radians.of(-Math.PI / 2), // front left
          Radians.of(0), // front right
          Radians.of(Math.PI), // rear left
          Radians.of(Math.PI / 2) // rear right
          );

  public static final class Translation {
    public static final double P = 0.6;
    public static final double I = 0.0;
    public static final double D = 0.0;
  }

  public static final class Rotation {
    public static final double P = 0.4;
    public static final double I = 0.0;
    public static final double D = 0.0;
  }

  public static final PathConstraints CONSTRAINTS =
      new PathConstraints(
          MAX_SPEED.in(MetersPerSecond),
          MAX_ACCEL.in(MetersPerSecondPerSecond),
          MAX_ANGULAR_SPEED.in(RadiansPerSecond),
          MAX_ANGULAR_ACCEL.in(RadiansPerSecond.per(Second)));

  public static final class SwerveModule {
    public static final class Driving {
      public static final Measure<Distance> CIRCUMFERENCE = Meters.of(2.0 * Math.PI * 0.0381);

      public static final Measure<Per<Angle, Angle>> GEARING =
          Rotations.of(1.0 / 45.0 / 22.0 * 15.0 * 14.0).per(Rotations);

      // cursed unit hack
      public static final Measure<Per<Distance, Angle>> CONVERSION =
          Meters.per(Radians).of(CIRCUMFERENCE.in(Meters) * GEARING.in(Radians.per(Radians)));

      public static final class PID {
        public static final double P = 0.11;
        public static final double I = 0.0;
        public static final double D = 0.06;
      }

      public static final class FF {
        public static final double S = 0.3;
        public static final double V = 2.7;
        public static final double A = 0.25;
      }
    }

    public static final class Turning {
      public static final Measure<Per<Angle, Angle>> MOTOR_GEARING =
          Rotations.of(1.0 / 4.0 / 3.0).per(Rotations);

      public static final Measure<Per<Angle, Angle>> CONVERSION = Radians.of(1).per(Rotations);

      public static final boolean ENCODER_INVERTED = true;

      public static final class PID {
        public static final double P = 2.0;
        public static final double I = 0.0;
        public static final double D = 0.1;
      }

      // system constants only used in simulation
      public static final class FF {
        public static final double S = 0.0;
        public static final double V = 0.25;
        public static final double A = 0.015;
      }
    }
  }
}
