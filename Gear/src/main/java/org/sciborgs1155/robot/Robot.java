import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.lib.DeferredCommand;
import org.sciborgs1155.robot.commands.Autos;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot extends CommandRobot implements Loggable {

  // INPUT DEVICES
  private final CommandXboxController operator = new CommandXboxController(OI.OPERATOR);
  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER);

  // SUBSYSTEMS

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

    autonomous().onTrue(getAutonomousCommand());
  }

  /**
   * Configures subsystem default commands. Default commands are scheduled when no other command is
   * running on a subsystem.
   */
  private void configureSubsystemDefaults() {}

  /** Configures trigger -> command bindings */
  private void configureBindings() {}

  /** The commamnd to be ran in autonomous */
  public Command getAutonomousCommand() {
    return new DeferredCommand(autos::get)
        .until(() -> !DriverStation.isAutonomous())
        .withName("auto");
  }
}
