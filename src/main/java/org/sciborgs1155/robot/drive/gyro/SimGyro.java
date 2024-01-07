package org.sciborgs1155.robot.drive.gyro;

public class SimGyro implements GyroIO {
    // TODO: somehow add values to yaw
    private double yaw;
    private double pitch;
    private double roll;

    public SimGyro(double yaw, double pitch, double roll) {
        this.yaw = yaw;
        this.pitch = pitch;
        this.roll = roll;
    }

    public SimGyro() {
        this(0, 0, 0);
    }

    public double getRate() {
        return 0; // ??
    }

    public double getYaw() {
        return yaw;
    }

    public double getPitch() {
        return pitch;
    }

    public double getRoll() {
        return roll;
    }

    public void reset() {
        yaw = 0;
        pitch = 0;
        roll = 0;
    }
}
