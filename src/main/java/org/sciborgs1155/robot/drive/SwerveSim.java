package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Kilograms;
import static org.sciborgs1155.robot.drive.DriveConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.sciborgs1155.lib.simulation.QuadSwerveSim;

public class SwerveSim implements SwerveIO {

  private final QuadSwerveSim simulation =
      new QuadSwerveSim(TRACK_WIDTH, TRACK_WIDTH, MASS.in(Kilograms), MOI, null);

  @Override
  public Pose2d getPose() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPose'");
  }

  @Override
  public void resetOdometry(Pose2d pose) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'resetOdometry'");
  }

  @Override
  public SwerveModuleState[] getModuleStates() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getModuleStates'");
  }

  @Override
  public SwerveModulePosition[] getModulePositions() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getModulePositions'");
  }

  @Override
  public Rotation3d getRotation() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getRotation'");
  }

  @Override
  public void close() throws Exception {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'close'");
  }
}
