package frc.robot;

import org.hotutilites.hotlogger.HotLogger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Arm.ArmStates;

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
    private boolean manual;
    private boolean lowPower;
    private boolean turnOnLimeLiteLight;
    private ArmStates armState;
    private double armDegreesFrom90;
    private double limelightHeight;
	private double highestClimberPos;
    private double driveVelocityLeft; //meters per second
    private double driveVelocityRight; //meters per second
    private boolean trajectoryComplete = false;

    public void resetRobotState(){
        theta = 0; // Degrees
        driveDistanceLeft = 0; // Meters
        driveDistanceRight = 0; // Meters
        driveVelocityLeft = 0; //meters per second
        driveVelocityRight = 0; //meters per second
        trajectoryComplete = false;
    }

    public double getTheta() {
        return theta;
    }

    public boolean isTurnOnLimeLiteLight() {
        return turnOnLimeLiteLight;
    }

    public void setTurnOnLimeLiteLight(boolean turnOnLimeLiteLight) {
        this.turnOnLimeLiteLight = turnOnLimeLiteLight;
    }

    public double getHighestClimberPos() {
        return highestClimberPos;
    }

    public void setHighestClimberPos(double highestClimberPos) {
        this.highestClimberPos = highestClimberPos;
    }

    public boolean isLowPower() {
        return lowPower;
    }

    public void setLowPower(boolean lowPower) {
        this.lowPower = lowPower;
    }

    public boolean isManual() {
        return manual;
    }
    public void setManual(boolean manual) {
        this.manual = manual;
    }
    public double getDistanceFromTarget() {
        return distanceFromTarget;
    }
    public void setDistanceFromTarget(double distanceFromTarget) {
        this.distanceFromTarget = distanceFromTarget;
        SmartDashboard.putNumber("distance from Target", distanceFromTarget);
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

    public int getVisionOutputStatus() {   // 3 = aimed
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
      
    }
    public double getDriveDistanceLeft() {
        return driveDistanceLeft;
    }
    public void setDriveDistanceLeft(double driveDistanceLeft) {
        this.driveDistanceLeft = driveDistanceLeft;
        HotLogger.Log("Drive_Distance_Left", driveDistanceLeft);
     
    }
    public void setTheta(double theta) {
        this.theta = theta;
        HotLogger.Log("theta", theta);
        SmartDashboard.putNumber("theta", theta);
    }
    public void setArmState(ArmStates armState){
        this.armState = armState;
        SmartDashboard.putString("arm state", armState.toString());
    }
    public void setArmAngleDegreesFrom90(double armDegreesFrom90) {
        this.armDegreesFrom90 = armDegreesFrom90;
    }
    
    public double getArmAngleDegreesFrom90() {
        SmartDashboard.putNumber("armAngle from 90", armDegreesFrom90);
        return armDegreesFrom90;
    }
    public double getLimelightHeight() {  //trig- opp = tangent*sin(armangle)
        limelightHeight = Calibrations.ARM.limelightHeightAtArm90 + (Calibrations.ARM.lengthOfArmToLimelight* Math.cos(Math.toRadians(90 - getArmAngleDegreesFrom90())));
        SmartDashboard.putNumber("Limelight height", limelightHeight);
        return limelightHeight;
       
    }

    public void setTrajectoryComplete(boolean trajectoryComplete) {
        this.trajectoryComplete = trajectoryComplete;
        SmartDashboard.putBoolean("trajComplete", trajectoryComplete);
    }
    
    public boolean getTrajectoryComplete() {
        return trajectoryComplete;
    }

    public void setDriveVelocityLeft(double driveVelocityLeft){ //m/s
        this.driveDistanceLeft = driveVelocityRight;
    }

    public void setDriveVelocityRight(double driveVelocityRight){  //m/s
        this.driveVelocityRight = driveVelocityRight;
    }

    public double getDriveVelocityLeft(){
        return driveVelocityLeft;
    }

    public double getDriveVelocityRight(){
        return driveVelocityRight;
    }
}