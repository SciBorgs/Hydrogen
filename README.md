# Hydrogen
The SciBorgs' base repository. It is a living document that should be updated yearly for new libraries and other changes.

## Structure
Our robot code is centered around [Robot.java](src/main/java/org/sciborgs1155/robot/Robot.java).

This project currently contains drive, vision, and autos. You are expected to add/modify code supporting existing files and new subsystems when using this template. Some of these files include but are not limited to:
- **[Autos.java](src/main/java/org/sciborgs1155/robot/commands/Autos.java)** Add code for new subsystems in `configureAutos`, such as commands for `NamedCommands`
- **[DriveConstants.java](src/main/java/org/sciborgs1155/robot/drive/DriveConstants.java)** Modify control constants yearly for each new robot, and all drivetrain constants for each new drivetrain as needed.
- **[VisionConstants.java](src/main/java/org/sciborgs1155/robot/vision/VisionConstants.java)** Add new `CameraConfig` fields representing cameras on the robot, and change the `create` method in [Vision](src/main/java/org/sciborgs1155/robot/vision/Vision.java). Also modify any camera configurations, AprilTag information, or constants if needed.
- **[Constants.java](src/main/java/org/sciborgs1155/robot/Constants.java)** Modify the class `Field` to be updated for each year's game, and any other constants if needed.
- **[Ports.java](src/main/java/org/sciborgs1155/robot/Ports.java)** Modify existing OI and drive ports, as well as adding new ports.
- **[Robot.java](src/main/java/org/sciborgs1155/robot/Robot.java)** A lot: subsystems, command files, related triggers & bindings, interactions with other subsystems & files, library configurations/starting, etc..

## Dependencies
- [WPILib](https://docs.wpilib.org/)
- [REVLib](https://docs.revrobotics.com/sparkmax/software-resources/spark-max-api-information)
- [PathPlannerLib](https://pathplanner.dev/home.html)
- [PhotonLib](https://docs.photonvision.org/en/latest/docs/programming/photonlib/adding-vendordep.html)
- [Phoenix6](https://v6.docs.ctr-electronics.com/en/stable/)
- [URCL](https://github.com/Mechanical-Advantage/URCL)
- [Studica](https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java/)
- [ReduxLib](https://docs.reduxrobotics.com/canandgyro/getting-started)
- [Spotless](https://github.com/diffplug/spotless/blob/main/plugin-gradle/README.md)
- [Monologue](https://github.com/shueja/Monologue)
