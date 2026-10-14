package org.sciborgs1155.lib.projectiles;

import static org.sciborgs1155.lib.LoggingUtils.log;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Simulates and displays projectiles (on network tables). Requires extensive implementation of
 * launch parameters.
 */
public abstract class ProjectileVisualizer {
  /** The first, second, and third values of a 3D vector correspond to X, Y, and Z. */
  public static final int X = 0, Y = 1, Z = 2;

  private double airTime;
  private int scores, misses;
  private boolean willScore, willMiss;

  private boolean launchEnabled = true;
  private boolean trajectoryEnabled = true;

  private boolean weightEnabled = true;
  private boolean dragEnabled = true;
  private boolean torqueEnabled = true;
  private boolean liftEnabled = true;

  private double launchResolution = 100;
  private double trajectoryResolution = 100;
  private double cooldown = 0.05;
  private double maxAirTime = 3.0;

  private int maxLaunchFrames, maxTrajectoryFrames;
  private double launchDt, trajectoryDt;
  private Pose3d initial, ending;

  private volatile Pose3d[] trajectory = new Pose3d[0];

  // INITIAL STATE SUPPLIERS
  private final DoubleSupplier initialTranslationX, initialTranslationY, initialTranslationZ;
  private final DoubleSupplier initialVelocityX, initialVelocityY, initialVelocityZ;
  private final DoubleSupplier initialRotationX, initialRotationY, initialRotationZ;
  private final DoubleSupplier initialRotationalVelocity;

  private final List<Projectile> projectiles = new ArrayList<>();

  /** Object pool for projectiles. */
  private final Deque<Projectile> projectilePool = new ArrayDeque<>();

  /** Reusable trajectory buffer. */
  private final List<Pose3d> trajectoryBuffer = new ArrayList<>(512);

  // CACHED LAUNCH STATE
  private double lastTranslationX, lastTranslationY, lastTranslationZ;
  private double lastVelocityX, lastVelocityY, lastVelocityZ;
  private double lastRotationX, lastRotationY, lastRotationZ;
  private double lastRotationalVelocity;

  private boolean launchStateInitialized;
  private volatile boolean trajectoryDirty = true;

  private final ScheduledExecutorService executor =
      Executors.newScheduledThreadPool(
          2, runnable -> new Thread(runnable, "Projectile Visualizer"));

  private final AtomicBoolean running = new AtomicBoolean(false);

  protected abstract Projectile createProjectile();

  protected ProjectileVisualizer(
      DoubleSupplier initialTranslationX,
      DoubleSupplier initialTranslationY,
      DoubleSupplier initialTranslationZ,
      DoubleSupplier initialVelocityX,
      DoubleSupplier initialVelocityY,
      DoubleSupplier initialVelocityZ,
      DoubleSupplier initialRotationX,
      DoubleSupplier initialRotationY,
      DoubleSupplier initialRotationZ,
      DoubleSupplier initialRotationalVelocity) {
    this.initialTranslationX = initialTranslationX;
    this.initialTranslationY = initialTranslationY;
    this.initialTranslationZ = initialTranslationZ;
    this.initialVelocityX = initialVelocityX;
    this.initialVelocityY = initialVelocityY;
    this.initialVelocityZ = initialVelocityZ;
    this.initialRotationX = initialRotationX;
    this.initialRotationY = initialRotationY;
    this.initialRotationZ = initialRotationZ;
    this.initialRotationalVelocity = initialRotationalVelocity;

    recomputeFrameLimits();
  }

  protected ProjectileVisualizer(
      CachedVector initialTranslation,
      CachedVector initialVelocity,
      CachedVector initialRotation,
      DoubleSupplier initialRotationalVelocity) {
    this.initialTranslationX = initialTranslation.x();
    this.initialTranslationY = initialTranslation.y();
    this.initialTranslationZ = initialTranslation.z();
    this.initialVelocityX = initialVelocity.x();
    this.initialVelocityY = initialVelocity.y();
    this.initialVelocityZ = initialVelocity.z();
    this.initialRotationX = initialRotation.x();
    this.initialRotationY = initialRotation.y();
    this.initialRotationZ = initialRotation.z();
    this.initialRotationalVelocity = initialRotationalVelocity;
  }

  private void recomputeFrameLimits() {
    maxLaunchFrames = (int) (maxAirTime * launchResolution);
    maxTrajectoryFrames = (int) (maxAirTime * trajectoryResolution);

    launchDt = 1.0 / launchResolution;
    trajectoryDt = 1.0 / trajectoryResolution;
  }

  private static boolean diff(double a, double b) {
    return Math.abs(a - b) > 1e-6;
  }

  private void checkLaunchState() {
    double tx = initialTranslationX.getAsDouble();
    double ty = initialTranslationY.getAsDouble();
    double tz = initialTranslationZ.getAsDouble();
    double vx = initialVelocityX.getAsDouble();
    double vy = initialVelocityY.getAsDouble();
    double vz = initialVelocityZ.getAsDouble();
    double rx = initialRotationX.getAsDouble();
    double ry = initialRotationY.getAsDouble();
    double rz = initialRotationZ.getAsDouble();
    double rv = initialRotationalVelocity.getAsDouble();

    boolean changed =
        !launchStateInitialized
            || diff(tx, lastTranslationX)
            || diff(ty, lastTranslationY)
            || diff(tz, lastTranslationZ)
            || diff(vx, lastVelocityX)
            || diff(vy, lastVelocityY)
            || diff(vz, lastVelocityZ)
            || diff(rx, lastRotationX)
            || diff(ry, lastRotationY)
            || diff(rz, lastRotationZ)
            || diff(rv, lastRotationalVelocity);

    if (changed) {
      lastTranslationX = tx;
      lastTranslationY = ty;
      lastTranslationZ = tz;
      lastVelocityX = vx;
      lastVelocityY = vy;
      lastVelocityZ = vz;
      lastRotationX = rx;
      lastRotationY = ry;
      lastRotationZ = rz;
      lastRotationalVelocity = rv;

      launchStateInitialized = true;
      trajectoryDirty = true;
    }
  }

  private Projectile obtainProjectile() {
    Projectile projectile = projectilePool.pollFirst();
    if (projectile == null) projectile = createProjectile();

    projectile.config(launchResolution, weightEnabled, dragEnabled, torqueEnabled, liftEnabled);

    return projectile;
  }

  private void recycleProjectile(Projectile projectile) {
    projectile.reset();
    projectilePool.addFirst(projectile);
  }

  /**
   * Starts the simulation thread. Note that once the thread has been started, no further changes
   * can be made to the simulation resolutions.
   */
  public void startSimulation() {
    if (!running.compareAndSet(false, true)) return;

    executor.scheduleAtFixedRate(
        this::updateLaunchSimulation, 0, (long) (launchDt * 1_000_000), TimeUnit.MICROSECONDS);

    executor.scheduleAtFixedRate(
        this::updateTrajectorySimulation,
        0,
        (long) (trajectoryDt * 1_000_000),
        TimeUnit.MICROSECONDS);
  }

  /**
   * Ends the simulation thread. Note that once the thread has been ended, it may not be started
   * again.
   */
  public void endSimulation() {
    running.set(false);
  }

  /**
   * Configures physics settings.
   *
   * @param weight whether or not to simulate gravity
   * @param drag whether or not to simulate drag
   * @param torque whether or not to simulate torque
   * @param lift whether or not to simulate lift (magnus force)
   */
  public ProjectileVisualizer configPhysics(
      boolean weight, boolean drag, boolean torque, boolean lift) {
    weightEnabled = weight;
    dragEnabled = drag;
    torqueEnabled = torque;
    liftEnabled = lift;

    trajectoryDirty = true;

    return this;
  }

  /**
   * Configures generation settings.
   *
   * @param delay the minimum delay between projectile launches. Requires launch mode to be enabled
   * @param air the maximum amount of time a projectile can spend in air before being deleted
   *     (seconds)
   * @param launch the resolution of the launched projectiles (steps per second). Requires launch
   *     mode to be enabled. Note that once the thread has been started, no further changes can be
   *     made to the simulation resolutions.
   * @param trajectory the resolution of the trajectory (steps per second). Requires trajectory mode
   *     to be enabled. Note that once the thread has been started, no further changes can be made
   *     to the simulation resolutions.
   */
  public ProjectileVisualizer configGeneration(
      double delay, double air, double launch, double trajectory) {
    cooldown = delay;
    maxAirTime = air;

    launchResolution = launch;
    trajectoryResolution = trajectory;

    recomputeFrameLimits();
    trajectoryDirty = true;

    return this;
  }

  /**
   * Configures whether or not launch mode/trajectory mode is enabled. Both can be enabled
   * simultaneously.
   *
   * @param launch Launch mode allows for the launching of individual projectiles in real time
   * @param trajectory Trajectory mode allows for the generation of a complete trajectory ahead of
   *     time (x50 more resource intensive)
   */
  public ProjectileVisualizer config(boolean launch, boolean trajectory) {
    launchEnabled = launch;
    trajectoryEnabled = trajectory;

    return this;
  }

  private void launchProjectile() {
    if (!running.get() || !launchEnabled) return;
    Projectile projectile = obtainProjectile();

    projectile.initialize(
        initialTranslationX.getAsDouble(),
        initialTranslationY.getAsDouble(),
        initialTranslationZ.getAsDouble(),
        initialVelocityX.getAsDouble(),
        initialVelocityY.getAsDouble(),
        initialVelocityZ.getAsDouble(),
        initialRotationX.getAsDouble(),
        initialRotationY.getAsDouble(),
        initialRotationZ.getAsDouble(),
        initialRotationalVelocity.getAsDouble());

    synchronized (projectiles) {
      projectiles.add(projectile);
    }

    initial = projectile.pose();
  }

  /** A command to launch projectiles continuously with a set delay. */
  public Command launchProjectiles() {
    return Commands.repeatingSequence(
        Commands.runOnce(this::launchProjectile).andThen(Commands.waitSeconds(cooldown)));
  }

  private void updateLaunchSimulation() {
    if (!launchEnabled) return;

    synchronized (projectiles) {
      for (int index = projectiles.size() - 1; index >= 0; index--) {
        Projectile projectile = projectiles.get(index);

        if (projectile.willMiss()) {
          misses++;
          ending = projectile.pose();
          projectiles.remove(index);
          recycleProjectile(projectile);
          continue;
        }

        if (projectile.willScore()) {
          scores++;
          ending = projectile.pose();
          projectiles.remove(index);
          recycleProjectile(projectile);
          continue;
        }

        if (projectile.frames >= maxLaunchFrames) {
          ending = projectile.pose();
          projectiles.remove(index);
          recycleProjectile(projectile);
          continue;
        }

        projectile.step();
      }
    }
  }

  private void updateTrajectorySimulation() {
    checkLaunchState();
    if (!trajectoryEnabled || !trajectoryDirty) return;

    trajectory = generateTrajectory();
    trajectoryDirty = false;
  }

  private Pose3d[] generateTrajectory() {
    trajectoryBuffer.clear();
    Projectile projectile = obtainProjectile();
    projectile.config(trajectoryResolution, weightEnabled, dragEnabled, torqueEnabled, liftEnabled);

    projectile.initialize(
        lastTranslationX,
        lastTranslationY,
        lastTranslationZ,
        lastVelocityX,
        lastVelocityY,
        lastVelocityZ,
        lastRotationX,
        lastRotationY,
        lastRotationZ,
        lastRotationalVelocity);

    initial = projectile.pose();

    int frames = 0;
    while (!projectile.willMiss() && !projectile.willScore() && frames <= maxTrajectoryFrames) {
      trajectoryBuffer.add(projectile.pose());
      projectile.step();
      frames++;
    }

    ending = projectile.pose();
    willMiss = projectile.willMiss();
    willScore = projectile.willScore();
    airTime = frames / trajectoryResolution;

    recycleProjectile(projectile);
    return trajectoryBuffer.toArray(new Pose3d[0]);
  }

  /** The poses of the projectiles being simulated. Requires launch mode to be enabled. */
  public Pose3d[] poses() {
    synchronized (projectiles) {
      Pose3d[] poses = new Pose3d[projectiles.size()];

      for (int index = 0; index < poses.length; index++)
        poses[index] = projectiles.get(index).pose();

      return poses;
    }
  }

  /**
   * The trajectory of the projectile when launched with the current parameters. Requires trajectory
   * to be enabled.
   */
  public Pose3d[] trajectory() {
    Pose3d[] output = new Pose3d[trajectory.length];
    System.arraycopy(trajectory, 0, output, 0, trajectory.length);
    return output;
  }

  /** The initial pose of the projectile. */
  public Pose3d initial() {
    return initial;
  }

  /** The final pose of the projectile. */
  public Pose3d ending() {
    return ending;
  }

  /**
   * Whether or not the projectile will hit the goal when launched with the current parameters.
   * Requires trajectory to be enabled.
   */
  public boolean willScore() {
    return willScore;
  }

  /**
   * Whether or not the projectile will miss the goal when launched with the current parameters.
   * Requires trajectory to be enabled.
   */
  public boolean willMiss() {
    return willMiss;
  }

  /**
   * The amount of time the projectile spends in the air (seconds). Requires trajectory to be
   * enabled.
   */
  public double airTime() {
    return airTime;
  }

  /**
   * The amount of time a projectile has been launched and scored into the goal. Requires launch to
   * be enabled.
   */
  public int scores() {
    return scores;
  }

  /**
   * The amount of time a projectile has been launched and missed the goal. Requires launch to be
   * enabled.
   */
  public int misses() {
    return misses;
  }

  /** Logs visualizer data to NetworkTables. */
  public void updateLogging() {
    log("Projectile Visualizer/Trajectory", trajectory(), Pose3d.struct);
    log("Projectile Visualizer/Will score", willScore);
    log("Projectile Visualizer/Will miss", willMiss);
    log("Projectile Visualizer/Air Time", airTime);
    log("Projectile Visualizer/Scores", scores);
    log("Projectile Visualizer/Misses", misses);
    log("Projectile Visualizer/Projectiles", poses(), Pose3d.struct);
    log("Projectile Visualizer/Launch pose", initial, Pose3d.struct);
    log("Projectile Visualizer/Ending pose", ending, Pose3d.struct);
  }

  /**
   * Caches a recomputed double[3] and exposes per-component DoubleSuppliers. Changes to the output
   * are made per call of the refresh method.
   */
  protected static final class CachedVector {
    private final Supplier<double[]> source;
    private final double[] cache = new double[3];
    private boolean dirty = true;

    public CachedVector(Supplier<double[]> source) {
      this.source = source;
    }

    private void refresh() {
      if (!dirty) return;
      double[] result = source.get();
      cache[0] = result[0];
      cache[1] = result[1];
      cache[2] = result[2];
      dirty = false;
    }

    public void invalidate() {
      dirty = true;
    }

    public DoubleSupplier x() {
      return () -> {
        refresh();
        return cache[X];
      };
    }

    public DoubleSupplier y() {
      return () -> {
        refresh();
        return cache[Y];
      };
    }

    public DoubleSupplier z() {
      return () -> {
        refresh();
        return cache[Z];
      };
    }
  }

  public static double norm(double[] vector) {
    double x = vector[X];
    double y = vector[Y];
    double z = vector[Z];

    return Math.sqrt(x * x + y * y + z * z);
  }

  public static double[] fromTranslation(Translation3d translation) {
    return new double[] {translation.getX(), translation.getY(), translation.getZ()};
  }
}
