package org.sciborgs1155.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;
import java.util.List;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.lib.failure.Fallible;
import org.sciborgs1155.lib.failure.FaultBuilder;
import org.sciborgs1155.lib.failure.HardwareFault;
import org.sciborgs1155.robot.Ports.OI;
import org.sciborgs1155.robot.commands.Autos;
import org.sciborgs1155.robot.drive.Drive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot extends CommandRobot implements Fallible, Loggable {

  // INPUT DEVICES
  private final CommandXboxController operator = new CommandXboxController(OI.OPERATOR);
  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER);

  // SUBSYSTEMS
  @Log Drive drive = Drive.create();

  // COMMANDS
  @Log Autos autos = new Autos();

  /** The robot contains subsystems, OI devices, and commands. */
  public Robot() {
    super(Constants.PERIOD);

    configureGameBehavior();
    configureBindings();
    configureSubsystemDefaults();
  }

  /** Configures basic behavior during different parts of the game. */
  private void configureGameBehavior() {
    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    Logger.configureLoggingAndConfig(this, false);

    DataLogManager.start();

    addPeriodic(Logger::updateEntries, Constants.PERIOD);

    autonomous().whileTrue(new ProxyCommand(autos::get));
  }

  /**
   * Configures subsystem default commands. Default commands are scheduled when no other command is
   * running on a subsystem.
   */
  private void configureSubsystemDefaults() {
    drive.setDefaultCommand(
        drive
            .drive(() -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX())
            .withName("teleop driving"));
  }

  /** Configures trigger -> command bindings */
  private void configureBindings() {
    // DRIVER INPUT
    driver.b().onTrue(drive.zeroHeading());
    driver.leftBumper().onTrue(drive.setSpeedMultiplier(0.3)).onFalse(drive.setSpeedMultiplier(1));
    driver.rightBumper().onTrue(drive.setSpeedMultiplier(0.3)).onFalse(drive.setSpeedMultiplier(1));

    // FAILING BEHAVIOR
    drive.onFailing(Commands.print("drive is failing!"));
  }

  @Override
  public List<HardwareFault> getFaults() {
    return FaultBuilder.create().register(drive.getFaults()).build();
  }
}
