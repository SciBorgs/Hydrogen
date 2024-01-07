package org.sciborgs1155.robot.drive.gyro;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

// PigeonIMU implementation
public class PigeonIMU implements GyroIO {
    private WPI_PigeonIMU pigeon;

    public PigeonIMU(int deviceId) {
        pigeon = new WPI_PigeonIMU(deviceId);
    }

    public double getRate() {
        return pigeon.getRate();
    }

    public double getYaw() {
        return pigeon.getYaw();
    }

    public double getPitch() {
        return pigeon.getPitch();
    }

    public double getRoll() {
        return pigeon.getRoll();
    }

    public void reset() {
        pigeon.reset();
    }
}
