package frc.robot;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
public class TrajectoryMaker{
    //sample auton path setpoints, this is where you should add other ones
                //possibly better to have different path classes for each path, or create pathmaker class using just start/end/rot as constructors for active use
  
    public double maxAcc;
    public double maxVel;
    Trajectory trajectory;
    TrajectoryConfig config;
    List<Translation2d> waypoints = new ArrayList<Translation2d>();
    public TrajectoryMaker(double MaxAcc, double MaxVel, double startRot, double startX, double startY, double endRot, double endX, double endY) {  //without waypoints, x is forward, positive y is left, neg is right
        maxAcc = MaxAcc;
        maxVel = MaxVel;
        
        TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAcc);
        //config start
        Rotation2d rotStart = new Rotation2d(Math.toRadians(startRot));   //degrees
        Translation2d XYStart = new Translation2d(startX, startY);  //x,y, should make field map with coordinate system to model
        Pose2d start = new Pose2d(XYStart, rotStart);
     
        //config end
        Rotation2d rotEnd = new Rotation2d(Math.toDegrees(endRot));   //degrees
        Translation2d XYEnd = new Translation2d(endX, endY);  //x,y, should make field map with coordinate system to model
        Pose2d end = new Pose2d(XYEnd, rotEnd);
        trajectory = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
    }

    public TrajectoryMaker(double MaxAcc, double MaxVel, double startRot, double startX, double startY, double endRot, double endX, double endY, double wayX, double wayY) {  //one waypoint. x is forward, positive y is left, neg is right
        maxAcc = MaxAcc;
        maxVel = MaxVel;
        
        
        TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAcc);
        //config start
        Rotation2d rotStart = new Rotation2d(Math.toRadians(startRot));   //degrees
        Translation2d XYStart = new Translation2d(startX, startY);  //x,y, should make field map with coordinate system to model
        Pose2d start = new Pose2d(XYStart, rotStart);
        //config waypoints
        Translation2d waypoint = new Translation2d(wayX, wayY);
        waypoints.add(waypoint);
        //config end
        Rotation2d rotEnd = new Rotation2d(Math.toRadians(endRot));   //degrees
        Translation2d XYEnd = new Translation2d(endX, endY);  //x,y, should make field map with coordinate system to model
        Pose2d end = new Pose2d(XYEnd, rotEnd);
        trajectory = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);  //add max velocity constraint
    }
    
    public TrajectoryMaker(double MaxAcc, double MaxVel, double startRot, double startX, double startY, double endRot, double endX, double endY, double wayX, double wayY, double wayX2, double wayY2) {  //one waypoint. x is forward, positive y is left, neg is right
        maxAcc = MaxAcc;
        maxVel = MaxVel;
        
        
        TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAcc);
        //config start
        Rotation2d rotStart = new Rotation2d(Math.toRadians(startRot));   //degrees
        Translation2d XYStart = new Translation2d(startX, startY);  //x,y, should make field map with coordinate system to model
        Pose2d start = new Pose2d(XYStart, rotStart);
        //config waypoints
        Translation2d waypoint = new Translation2d(wayX, wayY);
        waypoints.add(waypoint);
        Translation2d waypoint2 = new Translation2d(wayX2, wayY2);
        waypoints.add(waypoint2);
        //config end
        Rotation2d rotEnd = new Rotation2d(Math.toRadians(endRot));   //degrees
        Translation2d XYEnd = new Translation2d(endX, endY);  //x,y, should make field map with coordinate system to model
        Pose2d end = new Pose2d(XYEnd, rotEnd);
        trajectory = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);  //add max velocity constraint
    }
    public Trajectory getTrajectory(){
        return trajectory;
    }
    public Pose2d getPosAtTime(double time) { //libraries can also do vel, acc and curvature
        return trajectory.sample(time).poseMeters;
    }
}
