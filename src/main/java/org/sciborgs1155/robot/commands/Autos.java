package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.sciborgs1155.robot.Constants.alliance;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.drive.DriveConstants.Rotation;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;

public class Autos {
  private static Optional<Rotation2d> rotation = Optional.empty();

  public static SendableChooser<Command> configureAutos(Drive drive) {
    AutoBuilder.configureHolonomic(
        drive::pose,
        drive::resetOdometry,
        drive::getRobotRelativeChassisSpeeds,
        s -> drive.setChassisSpeeds(s, ControlMode.CLOSED_LOOP_VELOCITY),
        new HolonomicPathFollowerConfig(
            new PIDConstants(Translation.P, Translation.I, Translation.D),
            new PIDConstants(Rotation.P, Rotation.I, Rotation.D),
            DriveConstants.MAX_SPEED.in(MetersPerSecond),
            DriveConstants.RADIUS.in(Meters),
            new ReplanningConfig()),
        () -> alliance() == Alliance.Red,
        drive);

    PPHolonomicDriveController.setRotationTargetOverride(() -> rotation);

    SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
    chooser.addOption("no auto", Commands.none());
    return chooser;
  }
}
