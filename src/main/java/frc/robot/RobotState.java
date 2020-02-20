package frc.robot;

import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotState {
    private double theta; // Degrees
    private double driveDistanceLeft; // Meters
    private double driveDistanceRight; // Meters
    private int inventory;
    private boolean readyToShoot;
    private int shooterTargetRPM;
    private boolean fault;
    private boolean robotEnabled;
    private int VisionOutputStatus;
    private int LEDColorState;
    private boolean LEDFlash;
    private double limelightXTheta; // Degrees
    private double limelightYTheta; // Degrees
    private double distanceFromTarget; // Meters

    public double getTheta() {
        return theta;
    }

    public double getDistanceFromTarget() {
        return distanceFromTarget;
    }

    public void setDistanceFromTarget(double distanceFromTarget) {
        this.distanceFromTarget = distanceFromTarget;
    }

    public double getLimelightYTheta() {
        return limelightYTheta;
    }

    public void setLimelightYTheta(double limelightYTheta) {
        this.limelightYTheta = limelightYTheta;
    }

    public double getLimelightXTheta() {
        return limelightXTheta;
    }

    public void setLimelightXTheta(double limelightXTheta) {
        this.limelightXTheta = limelightXTheta;
    }

    public boolean isLEDFlash() {
        return LEDFlash;
    }

    public void setLEDFlash(boolean lEDFlash) {
        this.LEDFlash = lEDFlash;
    }

    public int getLEDColorState() {
        return LEDColorState;
    }

    public void setLEDColorState(int lEDColorState) {
        this.LEDColorState = lEDColorState;
    }

    public int getVisionOutputStatus() {
        return VisionOutputStatus;
    }

    public void setVisionOutputStatus(int visionOutputStatus) {
        this.VisionOutputStatus = visionOutputStatus;
    }

    public boolean isRobotEnabled() {
        return robotEnabled;
    }

    public void setRobotEnabled(boolean robotEnabled) {
        this.robotEnabled = robotEnabled;
    }

    public int getShooterTargetRPM() {
        return shooterTargetRPM;
    }

    public void setShooterTargetRPM(int shooterTargetRPM) {
        this.shooterTargetRPM = shooterTargetRPM;
    }

    public boolean isFault() {
        return fault;
    }

    public void setFault(boolean fault) {
        this.fault = fault;
    }

    public boolean isReadyToShoot() {
        return readyToShoot;
    }

    public void setReadyToShoot(boolean readyToShoot) {
        this.readyToShoot = readyToShoot;
    }

    public int getInventory() {
        return inventory;
    }

    public void setInventory(int inventory) {
        this.inventory = inventory;
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