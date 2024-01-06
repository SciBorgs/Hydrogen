package org.sciborgs1155.robot.drive.gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.SPI;

public class Navx implements GyroIO {
    private AHRS ahrs;
    
    public Navx() {
        ahrs = new AHRS(SPI.Port.kMXP); 
    }

    public double getRate() {
        return ahrs.getRate();
    }

    public Rotation3d getRotation3d() {
        return ahrs.getRotation3d();
    }

    public double getYaw() {
        return ahrs.getYaw();
    }

    public double getPitch() {
        return ahrs.getPitch();
    }

    public double getRoll() {
        return ahrs.getRoll();
    }

    public void reset() {
        ahrs.reset();
    }
}
