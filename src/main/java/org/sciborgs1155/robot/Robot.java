package org.sciborgs1155.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.List;
import monologue.Logged;
import monologue.Monologue;
import monologue.Monologue.LogBoth;
import monologue.Monologue.LogFile;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.lib.Fallible;
import org.sciborgs1155.lib.SparkUtils;
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
  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER);

  // SUBSYSTEMS
  @LogFile private final Drive drive = new Drive();

  // COMMANDS
  @LogBoth Autos autos = new Autos();

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
    drive.setDefaultCommand(drive.setSpeeds(driver::getLeftY, driver::getRightY));
  }

  /** Configures trigger -> command bindings */
  private void configureBindings() {
    autonomous().whileTrue(new ProxyCommand(autos::get));
    
    driver.y().and(driver.x()).whileTrue(drive.move(Drive.Direction.FORWARD_LEFT, 5));
    driver.y().and(driver.a()).whileTrue(drive.move(Drive.Direction.FORWARD_RIGHT, 5));
    driver.b().and(driver.x()).whileTrue(drive.move(Drive.Direction.BACKWARD_LEFT, 5));
    driver.b().and(driver.a()).whileTrue(drive.move(Drive.Direction.BACKWARD_RIGHT, 5));

    driver.y().and(driver.x().negate()).and(driver.a().negate()).whileTrue(drive.move(Drive.Direction.FORWARD, 10));
    driver.b().and(driver.x().negate()).and(driver.a().negate()).whileTrue(drive.move(Drive.Direction.BACKWARD, 10));
    driver.a().and(driver.y().negate()).and(driver.b().negate()).whileTrue(drive.move(Drive.Direction.TURN_RIGHT, 10));
    driver.x().and(driver.y().negate()).and(driver.b().negate()).whileTrue(drive.move(Drive.Direction.TURN_LEFT, 10));

    while (yIsPressed() && xIsPressed()) {
      drive.move(Drive.Direction.FORWARD_LEFT, 10);
    }
    while (yIsPressed() && aIsPressed()) {
      drive.move(Drive.Direction.FORWARD_RIGHT, 10);
    }
    while (bIsPressed() && xIsPressed()) {
      drive.move(Drive.Direction.BACKWARD_LEFT, 10);
    }
    while (bIsPressed() && aIsPressed()) {
      drive.move(Drive.Direction.BACKWARD_RIGHT, 10);
    }

    while (yIsPressed() && !(aIsPressed() || xIsPressed())) {
      drive.move(Drive.Direction.FORWARD, 10);
    }
    while (xIsPressed() && !(aIsPressed() || xIsPressed())) {
      drive.move(Drive.Direction.TURN_LEFT, 10);
    }
    while (aIsPressed() && !(aIsPressed() || xIsPressed())) {
      drive.move(Drive.Direction.TURN_RIGHT, 10);
    }
    while (bIsPressed() && !(aIsPressed() || xIsPressed())) {
      drive.move(Drive.Direction.BACKWARD, 10);
    }
    
    
    driver.a().whileTrue(drive.move(Drive.Direction.TURN_RIGHT, 10));
    driver.b().whileTrue(drive.move(Drive.Direction.BACKWARD, 10));
    driver.x().whileTrue(drive.move(Drive.Direction.TURN_LEFT, 10));
    driver.y().whileTrue(drive.move(Drive.Direction.FORWARD, 10));
  }

  @LogBoth public boolean aIsPressed() {
    return driver.a().getAsBoolean();
  }
  @LogBoth public boolean bIsPressed() {
    return driver.b().getAsBoolean();
  }
  @LogBoth public boolean xIsPressed() {
    return driver.x().getAsBoolean();
  }
  @LogBoth public boolean yIsPressed() {
    return driver.y().getAsBoolean();
  }

  @Override
  public List<Fault> getFaults() {
    return Fallible.from();
  }
}
