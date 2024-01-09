//"new" shooter here (basically doing the IO thingy)
package org.sciborgs1155.robot.Shooter;

import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Monologue.LogBoth;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.wpilibj.simulation.FlywheelSim;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class NewShooter extends SubsystemBase implements Logged {
  @LogBoth
  private final PIDController pidController = new PIDController(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);
  private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);
  @LogBoth
  private static double flywheelVelocity = 0;
  
  //lines 22-23,27-28 moved into Real and Sim Flywheel.java files

  public final FlywheelIO flywheel;
  public NewShooter(FlywheelIO flywheelIOtype) {
      flywheel = flywheelIOtype; //in theory, this sets a final flywheel to the correct type of object thing (like real fake no flywheels)
      // setDefaultCommand(
      // runOnce(
      //         () -> {
      //           motor.disable();
      //           //m_feederMotor.disable();
      //         })
      //     .andThen(run(() -> {}))
      //     .withName("Idle"));
  }

  public static NewShooter createFromConfigure(){
    //see if you are real
    return Robot.isReal() ? new NewShooter(new RealFlywheel()) : new NewShooter(new SimFlywheel());
  }

  public Command shootCommand(double setpointRPS) {
    // Run the shooter flywheel at the desired setpoint using feedforward and feedback
    return run(
      () ->
        flywheel.setVoltage(
          pidController.calculate(flywheel.getVelocity(), setpointRPS)
            + feedForward.calculate(setpointRPS)))
        // Wait until the shooter has reached the setpoint, and then run the feeder
    .withName("Shoot");
  }

  @LogBoth
  public boolean reallife() {
    return Robot.isReal(); //if else results moved into real and sim flywheels
  }

  @LogBoth
  @Override
  public void periodic() {
    flywheelVelocity=flywheel.getVelocity();
  }

  @Override
  public void simulationPeriodic() {
    if (Robot.isReal()){} 
      else{
        flywheel.tick();
    }
  }
}

//inspiration from https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/rapidreactcommandbot/subsystems/Shooter.java
//IO inspiration by Asa and https://github.com/SciBorgs/ChargedUp-2023/blob/main/src/main/java/org/sciborgs1155/robot/arm/ 