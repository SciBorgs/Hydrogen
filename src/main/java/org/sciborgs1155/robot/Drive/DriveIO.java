package org.sciborgs1155.robot.Drive;

public interface DriveIO {
    public void setSpeeds(double leftSpeed, double rightSpeed);

    public void periodic();

    public double getX();
    public double getY();

}
