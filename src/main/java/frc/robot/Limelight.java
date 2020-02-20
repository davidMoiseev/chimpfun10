package frc.robot;

import org.hotutilites.hotInterfaces.IHotSensor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight implements IHotSensor<RobotState, Double> {

    private NetworkTable limelight;
    private NetworkTableEntry tx, ty, tv, tl;
    private double canSeeTarget; // 0 is no, 1 is yes
    private RobotState robotState;

    private double xTheta, yTheta;
    private double latency;
    private double distanceFromTarget, sensorAngle;

    public Limelight(RobotState robotState){
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        tv = limelight.getEntry("tv");
        tl = limelight.getEntry("tl");
        this.robotState = robotState;
    }

    @Override
    public void updateState() {
        xTheta = tx.getDouble(0.0);
        yTheta = ty.getDouble(0.0);
        latency = tl.getDouble(0.0);
        canSeeTarget = tv.getDouble(0.0);
        robotState.setLimelightXTheta(xTheta);
        robotState.setLimelightYTheta(yTheta);
        robotState.setDistanceFromTarget(getDistanceFromTarget());

        if(latency > 1000){
            SmartDashboard.putNumber("visionOutputStatus", 0);
        }
        if(canSeeTarget > 1){
            SmartDashboard.putNumber("visionOutputStatus", 2);
        }else{
            SmartDashboard.putNumber("visionOutputStatus", 1);
        }
        if(xTheta > Calibrations.Vision.deadband){
            SmartDashboard.putNumber("visionOutputStatus", 3);
        } else {
            SmartDashboard.putNumber("visionOutputStatus", 2);
        }
    }
    
    @Override
    public void zeroSensor() {
        xTheta = tx.getDouble(0.0);
        yTheta = ty.getDouble(0.0);
        canSeeTarget = tv.getDouble(0.0);
    }

    public void setSensorValue(Double value) {
        xTheta = tx.getDouble(value);
        yTheta = ty.getDouble(value);
        canSeeTarget = tv.getDouble(value);
    }
    
    public void setSensorValue(double  x, double y, double canSee){
        xTheta = tx.getDouble(x);
        yTheta = ty.getDouble(y);
        canSeeTarget = tv.getDouble(0.0);
    }

    public double getDistanceFromTarget(){
        sensorAngle = yTheta;
        distanceFromTarget = (Calibrations.Vision.kHeight / (Math.tan(Math.toRadians(sensorAngle + Calibrations.Vision.kMountedAngle)))) - Calibrations.Vision.kLimelightDistanceFromFront;
        return distanceFromTarget;
    }

    // public double config(){
    //     kMountedAngle = Math.toDegrees(Math.atan(kHeight / 2.32)) - sensorAngle; // 2.12 = distance from target, change later for real robot
    //     return kMountedAngle;
        
    //}

    @Override
    public void setRobotState(RobotState robotState) {
        this.robotState = robotState;
    }
}