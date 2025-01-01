package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import java.util.List;

/**
 * Constants for our 2024 MAXSwerve drivetrain. All fields in this file should be updated for the
 * current robot configuration!
 */
public final class DriveConstants {
  /** The type of control loop to use when controlling a module's drive motor. */
  public static enum ControlMode {
    CLOSED_LOOP_VELOCITY,
    OPEN_LOOP_VELOCITY;
  }

  /** The type of modules being used. */
  public static enum ModuleType {
    TALON, // Kraken X60 Drive, NEO 550 Turn
    SPARK; // NEO Vortex Drive, NEO 550 Turn
  }

  // TODO: Change central drivetrain constants as needed.

  // The type of module on the chassis
  public static final ModuleType TYPE = ModuleType.SPARK;

  // The control loop used by all of the modules when driving
  public static final ControlMode DRIVE_MODE = ControlMode.OPEN_LOOP_VELOCITY;

  // Rate at which sensors update periodicially
  public static final Time SENSOR_PERIOD = Seconds.of(0.02);

  // Distance between centers of right and left wheels on robot
  public static final Distance TRACK_WIDTH = Meters.of(0.5715);
  // Distance between front and back wheels on robot
  public static final Distance WHEEL_BASE = Meters.of(0.5715);
  // The radius of any swerve wheel
  public static final Distance WHEEL_RADIUS = Inches.of(1.5);
  // Distance from the center to any wheel of the robot
  public static final Distance RADIUS = TRACK_WIDTH.div(2).times(Math.sqrt(2));
  // Coefficient of friction between the drive wheel and the carpet.
  public static final double WHEEL_COF = 1.0;
  // Robot width with bumpers
  public static final Distance CHASSIS_WIDTH = Inches.of(32.645);

  // Maximum achievable translational and rotation velocities and accelerations of the robot.
  public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(5.74);
  public static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(16.0);
  public static final AngularVelocity MAX_ANGULAR_SPEED =
      RadiansPerSecond.of(MAX_SPEED.in(MetersPerSecond) / RADIUS.in(Meters));
  public static final AngularAcceleration MAX_ANGULAR_ACCEL =
      RadiansPerSecond.per(Second).of(MAX_ACCEL.in(MetersPerSecondPerSecond) / RADIUS.in(Meters));

  // Arbitrary max rotational velocity for the driver to effectively control the robot
  public static final AngularVelocity TELEOP_ANGULAR_SPEED = Radians.per(Second).of(2 * Math.PI);

  public static final Translation2d[] MODULE_OFFSET = {
    new Translation2d(WHEEL_BASE.div(2), TRACK_WIDTH.div(2)), // front left
    new Translation2d(WHEEL_BASE.div(2), TRACK_WIDTH.div(-2)), // front right
    new Translation2d(WHEEL_BASE.div(-2), TRACK_WIDTH.div(2)), // rear left
    new Translation2d(WHEEL_BASE.div(-2), TRACK_WIDTH.div(-2)) // rear right
  };

  // angular offsets of the modules, since we use absolute encoders
  // ignored (used as 0) in simulation because the simulated robot doesn't have offsets
  public static final List<Rotation2d> ANGULAR_OFFSETS =
      List.of(
          Rotation2d.fromRadians(-Math.PI / 2), // front left
          Rotation2d.fromRadians(0), // front right
          Rotation2d.fromRadians(Math.PI), // rear left
          Rotation2d.fromRadians(Math.PI / 2) // rear right
          );

  public static final Rotation3d GYRO_OFFSET = new Rotation3d(0, 0, Math.PI);

  // TODO: Change ALL characterization constants for each unique robot as needed.
  public static final class Translation {
    public static final double P = 3.0;
    public static final double I = 0.0;
    public static final double D = 0.05;

    public static final Distance TOLERANCE = Centimeters.of(5);
  }

  public static final class Rotation {
    public static final double P = 4.5;
    public static final double I = 0.0;
    public static final double D = 0.05;

    public static final Angle TOLERANCE = Degrees.of(3);
  }

  public static final class ModuleConstants {
    public static final double COUPLING_RATIO = 0;

    public static final class Driving {
      // Possible pinion configurations : 12T, 13T, or 14T.
      public static final int PINION_TEETH = 14;

      public static final Distance CIRCUMFERENCE = Meters.of(2.0 * Math.PI * 0.0381);

      // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
      // bevel pinion
      public static final double GEARING = 1.0 / 45.0 / 22.0 * 15.0 * 14.0;

      public static final Distance POSITION_FACTOR = CIRCUMFERENCE.times(GEARING);
      public static final LinearVelocity VELOCITY_FACTOR = POSITION_FACTOR.per(Minute);

      public static final Current CURRENT_LIMIT = Amps.of(50);

      public static final class PID {
        public static final class SPARK {
          public static final double P = 3.2;
          public static final double I = 0.0;
          public static final double D = 0.0;
        }

        public static final class TALON {
          public static final double P = 3.2;
          public static final double I = 0.0;
          public static final double D = 0.0;
        }
      }

      public static final class FF {
        public static final class SPARK {
          public static final double S = 0.088468;
          public static final double V = 2.1314;
          public static final double A = 0.33291;
        }

        public static final class TALON {
          public static final double S = 0.088468;
          public static final double V = 2.1314;
          public static final double A = 0.33291;
        }
      }
    }

    static final class Turning {
      public static final double MOTOR_GEARING = 1.0 / 4.0 / 3.0;
      public static final double ENCODER_GEARING = 1;

      public static final Angle POSITION_FACTOR = Rotations.of(ENCODER_GEARING);
      public static final AngularVelocity VELOCITY_FACTOR = POSITION_FACTOR.per(Minute);

      public static final boolean ENCODER_INVERTED = true;

      public static final Current CURRENT_LIMIT = Amps.of(20);

      public static final class PID {
        public static final double P = 9;
        public static final double I = 0.0;
        public static final double D = 0.05;
      }

      // system constants only used in simulation
      public static final class FF {
        public static final double S = 0.30817;
        public static final double V = 0.55;
        public static final double A = 0.03;
      }
    }
  }
}
