package org.sciborgs1155.lib.projectiles;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.function.Predicate;
import java.util.function.ToDoubleFunction;

/** A single simulated projectile. RK4 integration courtesy of Claude! */
public class Projectile {
  /** The first, second, and third values of a 3D vector correspond to X, Y, and Z. */
  public static final int X = 0, Y = 1, Z = 2;

  public static final double GRAVITY = -9.80665;
  public static final double AIR_DENSITY = 1.225;

  protected int frames;
  protected double resolution, delta, halfDelta, dt6;
  protected boolean weightEnabled, dragEnabled, torqueEnabled, liftEnabled;
  protected double x, y, z;
  protected double vx, vy, vz;
  protected double ax, ay, az;
  protected double roll, pitch, yaw;
  protected double omega;
  protected double alpha;

  // Pre-allocated RK4 working buffers — never allocate in step()
  private final double[] s0 = new double[8];
  private final double[] k1 = new double[8];
  private final double[] k2 = new double[8];
  private final double[] k3 = new double[8];
  private final double[] k4 = new double[8];
  private final double[] tmp = new double[8];

  // Pre-allocated force output buffers
  private final double[] dragOut = new double[3];
  private final double[] liftOut = new double[3];

  // Reusable state carrier — never escapes derivatives()
  private final MutableState stateCarrier = new MutableState();

  // Defers the 5th derivatives() call until ax/ay/az/alpha are actually read
  private boolean accelDirty = true;

  // Strategy functions with no-op defaults
  private ForceFunction dragFunction =
      (s, out) -> {
        out[0] = 0;
        out[1] = 0;
        out[2] = 0;
      };
  private ForceFunction liftFunction =
      (s, out) -> {
        out[0] = 0;
        out[1] = 0;
        out[2] = 0;
      };
  private ToDoubleFunction<MutableState> torqueFunction = s -> 0.0;
  private Predicate<MutableState> scoreFunction = s -> false;
  private Predicate<MutableState> missFunction = s -> false;

  /**
   * Mutable snapshot of projectile state passed to force functions. Reused on every step to avoid
   * per-step allocation.
   */
  public final class MutableState {
    public double x, y, z, vx, vy, vz, pitch, omega, roll, yaw;

    /** Loads from an RK4 intermediate state array. Roll and yaw are unchanged during a step. */
    void load(double[] s) {
      x = s[0];
      y = s[1];
      z = s[2];
      vx = s[3];
      vy = s[4];
      vz = s[5];
      pitch = s[6];
      omega = s[7];
      roll = Projectile.this.roll;
      yaw = Projectile.this.yaw;
    }

    /** Loads directly from the projectile's current field state. */
    void loadCurrent() {
      x = Projectile.this.x;
      y = Projectile.this.y;
      z = Projectile.this.z;
      vx = Projectile.this.vx;
      vy = Projectile.this.vy;
      vz = Projectile.this.vz;
      pitch = Projectile.this.pitch;
      omega = Projectile.this.omega;
      roll = Projectile.this.roll;
      yaw = Projectile.this.yaw;
    }
  }

  /**
   * Computes a force vector given the current state. Writes {ax, ay, az} into the provided output
   * buffer rather than allocating a new array.
   */
  @FunctionalInterface
  public interface ForceFunction {
    void apply(MutableState state, double[] out);
  }

  public Projectile config(double fps, boolean weight, boolean drag, boolean torque, boolean lift) {
    resolution = fps;
    delta = 1.0 / fps;
    halfDelta = 0.5 * delta;
    dt6 = delta / 6.0;
    weightEnabled = weight;
    dragEnabled = drag;
    torqueEnabled = torque;
    liftEnabled = lift;

    return this;
  }

  public Projectile withDrag(ForceFunction f) {
    dragFunction = f;
    return this;
  }

  public Projectile withLift(ForceFunction f) {
    liftFunction = f;
    return this;
  }

  public Projectile withTorque(ToDoubleFunction<MutableState> f) {
    torqueFunction = f;
    return this;
  }

  public Projectile withScoreCondition(Predicate<MutableState> f) {
    scoreFunction = f;
    return this;
  }

  public Projectile withMissCondition(Predicate<MutableState> f) {
    missFunction = f;
    return this;
  }

  public double x() {
    return x;
  }

  public double y() {
    return y;
  }

  public double z() {
    return z;
  }

  /** Recomputes acceleration lazily — only if the state has changed since last read. */
  public double ax() {
    if (accelDirty) refreshAccel();
    return ax;
  }

  public double ay() {
    if (accelDirty) refreshAccel();
    return ay;
  }

  public double az() {
    if (accelDirty) refreshAccel();
    return az;
  }

  public double alpha() {
    if (accelDirty) refreshAccel();
    return alpha;
  }

  public boolean willScore() {
    stateCarrier.loadCurrent();
    return scoreFunction.test(stateCarrier);
  }

  public boolean willMiss() {
    stateCarrier.loadCurrent();
    return missFunction.test(stateCarrier);
  }

  /** Replaces NaN with 0.0 so invalid supplier values are handled gracefully. */
  protected static double validated(double value) {
    return value == value ? value : 0.0;
  }

  protected void initialize(
      double tx,
      double ty,
      double tz,
      double ivx,
      double ivy,
      double ivz,
      double rx,
      double ry,
      double rz,
      double rotationalVelocity) {
    x = validated(tx);
    y = validated(ty);
    z = validated(tz);
    vx = validated(ivx);
    vy = validated(ivy);
    vz = validated(ivz);
    ax = 0;
    ay = 0;
    az = 0;
    roll = validated(rx);
    pitch = validated(ry);
    yaw = validated(rz);
    omega = validated(rotationalVelocity);
    alpha = 0;
    frames = 0;
    accelDirty = true;
  }

  public void initialize(
      double[] translation,
      double[] initialVelocity,
      double[] initialRotation,
      double rotationalVelocity) {
    initialize(
        translation[X],
        translation[Y],
        translation[Z],
        initialVelocity[X],
        initialVelocity[Y],
        initialVelocity[Z],
        initialRotation[X],
        initialRotation[Y],
        initialRotation[Z],
        rotationalVelocity);
  }

  /**
   * Advances the simulation by one timestep using RK4.
   *
   * <p>State vector: [x, y, z, vx, vy, vz, pitch, omega] Derivative vector: [vx, vy, vz, ax, ay,
   * az, omega, alpha]
   */
  public void step() {
    s0[0] = x;
    s0[1] = y;
    s0[2] = z;
    s0[3] = vx;
    s0[4] = vy;
    s0[5] = vz;
    s0[6] = pitch;
    s0[7] = omega;

    derivatives(s0, k1);
    advanceState(s0, k1, halfDelta, tmp);
    derivatives(tmp, k2);
    advanceState(s0, k2, halfDelta, tmp);
    derivatives(tmp, k3);
    advanceState(s0, k3, delta, tmp);
    derivatives(tmp, k4);

    x = s0[0] + dt6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
    y = s0[1] + dt6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
    z = s0[2] + dt6 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
    vx = s0[3] + dt6 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]);
    vy = s0[4] + dt6 * (k1[4] + 2 * k2[4] + 2 * k3[4] + k4[4]);
    vz = s0[5] + dt6 * (k1[5] + 2 * k2[5] + 2 * k3[5] + k4[5]);
    pitch = s0[6] + dt6 * (k1[6] + 2 * k2[6] + 2 * k3[6] + k4[6]);
    omega = s0[7] + dt6 * (k1[7] + 2 * k2[7] + 2 * k3[7] + k4[7]);

    accelDirty = true;
    frames++;
  }

  /** Evaluates the derivative vector for a given state, writing the result into {@code out}. */
  private void derivatives(double[] s, double[] out) {
    stateCarrier.load(s);

    double dax = 0, day = 0, daz = 0;

    if (weightEnabled) daz += GRAVITY;

    if (dragEnabled) {
      dragFunction.apply(stateCarrier, dragOut);
      dax += dragOut[0];
      day += dragOut[1];
      daz += dragOut[2];
    }

    if (liftEnabled) {
      liftFunction.apply(stateCarrier, liftOut);
      dax += liftOut[0];
      day += liftOut[1];
      daz += liftOut[2];
    }

    double dalpha = torqueEnabled ? torqueFunction.applyAsDouble(stateCarrier) : 0.0;

    out[0] = s[3];
    out[1] = s[4];
    out[2] = s[5]; // dx/dt  = velocity
    out[3] = dax;
    out[4] = day;
    out[5] = daz; // dv/dt  = acceleration
    out[6] = s[7];
    out[7] = dalpha; // dpitch/dt = omega, domega/dt = alpha
  }

  /**
   * Recomputes ax/ay/az/alpha from the current state. Called at most once per step, and only if
   * those values are actually read.
   */
  private void refreshAccel() {
    s0[0] = x;
    s0[1] = y;
    s0[2] = z;
    s0[3] = vx;
    s0[4] = vy;
    s0[5] = vz;
    s0[6] = pitch;
    s0[7] = omega;
    derivatives(s0, k1);
    ax = k1[3];
    ay = k1[4];
    az = k1[5];
    alpha = k1[7];
    accelDirty = false;
  }

  /** Computes {@code out = s + scale * k} in-place. */
  private static void advanceState(double[] s, double[] k, double scale, double[] out) {
    for (int i = 0; i < 8; i++) out[i] = s[i] + scale * k[i];
  }

  public Pose3d pose() {
    return new Pose3d(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw));
  }

  public void reset() {
    x = 0;
    y = 0;
    z = 0;
    vx = 0;
    vy = 0;
    vz = 0;
    ax = 0;
    ay = 0;
    az = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;
    omega = 0;
    alpha = 0;
    frames = 0;
    accelDirty = true;
  }
}
