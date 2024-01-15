package org.sciborgs1155.robot;

import static org.sciborgs1155.robot.Constants.*;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.List;
import monologue.Logged;
import monologue.Monologue;
import monologue.Monologue.LogBoth;
import monologue.Monologue.LogFile;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.lib.Fallible;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.robot.Hopper.*;
import org.sciborgs1155.robot.Intake.*;
import org.sciborgs1155.robot.Ports.OI;
import org.sciborgs1155.robot.commands.Autos;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot extends CommandRobot implements Logged, Fallible {

  // INPUT DEVICES
  private final CommandXboxController operator = new CommandXboxController(OI.OPERATOR);

  // SUBSYSTEMS
  @LogFile private final Intake intake = new Intake();
  @LogFile private final Hopper hopper = new Hopper();

  // COMMANDS
  @LogBoth Autos autos = new Autos();

  @LogBoth
  boolean xIsPressed() {
    return operator.x().getAsBoolean();
  }
  ;

  /** The robot contains subsystems, OI devices, and commands. */
  public Robot() {
    super(Constants.PERIOD);

    configureGameBehavior();
    configureSubsystemDefaults();
    configureBindings();
  }

  /** Configures basic behavior during different parts of the game. */
  private void configureGameBehavior() {
    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    // Configure logging with DataLogManager and Monologue
    DataLogManager.start();
    Monologue.setupLogging(this, "/Robot");
    addPeriodic(Monologue::update, kDefaultPeriod);

    // Burn flash of all Spark Max at once with delays
    SparkUtils.safeBurnFlash();
  }

  /**
   * Configures subsystem default commands. Default commands are scheduled when no other command is
   * running on a subsystem.
   */
  private void configureSubsystemDefaults() {
    hopper.setDefaultCommand(hopper.stop());
    intake.setDefaultCommand(intake.stop());
  }

  /** Configures trigger -> command bindings */
  double
      launchAngle; // find from method written to optimize time of travel for a given distance with

  // variables angle and speed

  private void configureBindings() {
    operator.x().whileTrue(hopper.forward());
  }

  @Override
  public List<Fault> getFaults() {
    return Fallible.from();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
