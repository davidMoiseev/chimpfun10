package frc.robot;

import org.hotutilites.hotInterfaces.IRobotState;
import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotState implements IRobotState {
    private double theta; // Degrees
    private double driveDistanceLeft; // Meters
    private double driveDistanceRight; // Meters

    public double getTheta() {
        return theta;
    }

    public double getDriveDistanceRight() {
        return driveDistanceRight;
    }

    public void setDriveDistanceRight(double driveDistanceRight) {
        this.driveDistanceRight = driveDistanceRight;
        HotLogger.Log("Drive_Distance_Right", driveDistanceRight);
        SmartDashboard.putNumber("Drive_Distance_Right", driveDistanceRight);
    }

    public double getDriveDistanceLeft() {
        return driveDistanceLeft;
    }

    public void setDriveDistanceLeft(double driveDistanceLeft) {
        this.driveDistanceLeft = driveDistanceLeft;
        HotLogger.Log("Drive_Distance_Left", driveDistanceLeft);
        SmartDashboard.putNumber("Drive_Distance_Left", driveDistanceLeft);
    }

    public void setTheta(double theta) {
        this.theta = theta;
        HotLogger.Log("theta", theta);
        SmartDashboard.putNumber("theta", theta);
    }
}