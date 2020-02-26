package frc.robot;
import org.hotutilites.hotInterfaces.IHotSensor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class Limelight implements IHotSensor<RobotState, Double> {
    private NetworkTable limelight;
    private NetworkTableEntry tx, ty, tv, tl, ledMode;
    private double canSeeTarget; // 0 is no, 1 is yes
    private RobotState robotState;
    private double xTheta, yTheta;
    private double latency;
    private double distanceFromTarget, sensorAngle;
    private double cameraAngle;
    public Limelight(RobotState robotState){
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        tv = limelight.getEntry("tv");
        tl = limelight.getEntry("tl");
        ledMode = limelight.getEntry("ledMode");
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
        
        if (robotState.isRobotEnabled()){
            ledMode.setNumber(3);
        } else {
            ledMode.setNumber(1);
        }
        if(latency > 1000){
            SmartDashboard.putNumber("VisionOutputStatus", 0);
            robotState.setVisionOutputStatus(0);
        }else if(Math.abs(xTheta) < Calibrations.Vision.deadband + 1 && canSeeTarget >= 1){
            SmartDashboard.putNumber("VisionOutputStatus", 3);
            robotState.setVisionOutputStatus(3);
        }else if(canSeeTarget == 1){
            SmartDashboard.putNumber("VisionOutputStatus", 2);
            robotState.setVisionOutputStatus(2);
        }else{
            SmartDashboard.putNumber("VisionOutputStatus", 1);
            robotState.setVisionOutputStatus(1);
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
        //distanceFromTarget = (Calibrations.Vision.kHeight / (Math.tan(Math.toRadians(sensorAngle + Calibrations.Vision.kMountedAngle)))) - Calibrations.Vision.kLimelightDistanceFromFront;
        cameraAngle = 90 - (180 - 90 - (90 - robotState.getArmAngleDegreesFrom90())); 
        SmartDashboard.putNumber("camera angle",  cameraAngle);
        distanceFromTarget = ((2.7114 - robotState.getLimelightHeight()) / (Math.tan(Math.toRadians(sensorAngle + cameraAngle)))) - Calibrations.Vision.kLimelightDistanceFromFront;
        return distanceFromTarget;
    }
    // public double config(){  //using distance to calc mounted angle
    //     kMountedAngle = Math.toDegrees(Math.atan(kHeight / 2.32)) - sensorAngle; // 2.12 = distance from target, change later for real robot
    //     return kMountedAngle;
        
    //}
    @Override
    public void setRobotState(RobotState robotState) {
        this.robotState = robotState;
    }
}