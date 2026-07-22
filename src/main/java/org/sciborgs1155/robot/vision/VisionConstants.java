package org.sciborgs1155.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Set;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.sciborgs1155.robot.FieldConstants;
import org.sciborgs1155.robot.vision.Vision.CameraConfig;

public final class VisionConstants {
  public static final AprilTagFieldLayout TAG_LAYOUT = FieldConstants.FIELD_LAYOUT;

  // The PoseStrategy in multitag mode when only one tag is seen. Do NOT use MULTI_TAG_PNP varients.
  public static final PoseStrategy SINGLE_TAG_FALLBACK = PoseStrategy.LOWEST_AMBIGUITY;

  /** TODO: Create cameras with updated constants; be sure to add in {@link Vision#create} */
  // WARNING: EMPTY TRANSFORMS WILL CRASH SIMULATION UPON TAG DETECTION
  // TODO: actually add camera positions, figure out if its actually 148 fov
  public static final CameraConfig FL_CAMERA =
      new CameraConfig(
          "FL cam",
          78,
          new Transform3d(Inches.of(1), Inches.of(1), Inches.of(1), yawPitchRoll(90, -20, 180)),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);

  public static final CameraConfig FR_CAMERA =
      new CameraConfig(
          "FR cam",
          78,
          new Transform3d(Inches.of(1), Inches.of(-1), Inches.of(1), yawPitchRoll(-90, -20, 180)),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);

  // Camera constants for our configuration
  public static final int WIDTH = 1280;
  public static final int HEIGHT = 720;

  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS =
      VecBuilder.fill(0.9, 0.9, 1155); // TODO decide these later when we test the bump
  public static final Matrix<N3, N1> MULTIPLE_TAG_STD_DEVS = VecBuilder.fill(0.6, 0.6, 1155);
  public static final Matrix<N3, N1> SUPERTRUST_TAG_STD_DEVS = VecBuilder.fill(0.01, 0.01, 0.01);

  public static final double MAX_HEIGHT = 0.305;
  public static final double MAX_ANGLE = Math.PI;
  public static final double MAX_AMBIGUITY = 0.20;
  public static final double MAX_DISTANCE = FieldConstants.LENGTH.in(Meters) / 2.0;

  /** TODO: Modify AprilTag information as needed. */
  // Total of n AprilTags
  // Reference:
  // Tag Locations (1-n) | Description...

  public static final double[] TAG_WEIGHTS = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
  };

  public static final Set<Integer> UNREPUTABLE_TAGS = Set.of();

  /**
   * Returns a {@link Rotation3d} that represents a camera rotation, given the yaw, pitch, and roll.
   */
  public static Rotation3d yawPitchRoll(
      double yawDegrees, double pitchDegrees, double rollDegrees) {
    return new Rotation3d(
        Degrees.of(rollDegrees), Degrees.of(pitchDegrees), Degrees.of(yawDegrees));
  }
}
