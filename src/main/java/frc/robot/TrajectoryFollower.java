package frc.robot;

import frc.robot.TrajectoryMaker;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.controller.RamseteController;

import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class TrajectoryFollower {


    public static final double loopsPerUpdate = 3;

    public double rightOutput;
    public double leftOutput;
    private int counter;

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Calibrations.DRIVE_CONSTANTS.trackWidth);
    private final DifferentialDriveOdometry odometry;

    TrajectoryMaker shootingStartPosToWofFront;


    Trajectory currentTraj;
    State currentStateGoal;
    Pose2d currentPos;
    Pose2d origin = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    RamseteController controller;
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds();
    ChassisSpeeds adjustedSpeeds = new ChassisSpeeds();
    public double trajDuration = 0.001;
    Timer trajTimer = new Timer();

    private boolean previouslyReportedFinished;


    //paths
    public TrajectoryFollower(double yaw) {
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(yaw));  //initializes with position of 0,0,0. Add in for different starting
        generateTrajectories();

    }

    public enum PathNames {
        shootingStartPosToWofFront,
        sample2,
        none
    }

    public void generateTrajectories() {  //structured so all auton trajs can be generated when robot inits to save time
        shootingStartPosToWofFront = new TrajectoryMaker(1.8, 1.8, 0, 0, 0, 0, 3.5, 0);    // 4.13, 0.7 //(2.0, 2, 0, 0, 0, 0, 4, 0.5)

    }

    public void startTrajectory(PathNames trajName, double yaw) {    //structured so startTrajectory(name) can be called for any for auton and such (precalculated trajs)

        switch (trajName) {
            case shootingStartPosToWofFront:
                currentTraj = shootingStartPosToWofFront.getTrajectory();
                SmartDashboard.putBoolean("correct sample", true);


            case none:
                SmartDashboard.putBoolean("no sample", true);
                break;
        }


        trajTimer = new Timer();


        odometry.resetPosition(origin, Rotation2d.fromDegrees(yaw));
        controller = new RamseteController(Calibrations.AUTO_CONTROLLERS.ramseteB, Calibrations.AUTO_CONTROLLERS.ramseteTheta);
        trajDuration = currentTraj.getTotalTimeSeconds();
        SmartDashboard.putNumber("traj duration", trajDuration);
        trajTimer.start();
        counter = 1;


    }

    public boolean isTrajFinished() {
        if ((Math.abs(leftOutput) < 0.001) && (Math.abs(rightOutput) < 0.001) && (trajTimer.get() > trajDuration) && !previouslyReportedFinished) {
            previouslyReportedFinished = true;
            return true;
        } else {
            previouslyReportedFinished = false;
            return false;
        }
    }

    public void update(double currentYaw, double leftEncoderDistance, double rightEncoderDistance) {


        if ((counter != 0) && ((counter % loopsPerUpdate == 0) || (counter == 1))) {
            odometry.update(Rotation2d.fromDegrees(currentYaw), leftEncoderDistance, rightEncoderDistance);
            currentPos = odometry.getPoseMeters();  //working
            currentStateGoal = currentTraj.sample(trajTimer.get());  //working
            adjustedSpeeds = controller.calculate(currentPos, currentStateGoal);  //0
            wheelSpeeds = kinematics.toWheelSpeeds(adjustedSpeeds);


            // SmartDashboard.putString("path position now", currentStateGoal.poseMeters.toString());
            // SmartDashboard.putNumber ("state goal velocity now", currentStateGoal.velocityMetersPerSecond);
            // SmartDashboard.putString("current position", currentPos.toString());

        }

        counter++;


        leftOutput = wheelSpeeds.leftMetersPerSecond;
        rightOutput = wheelSpeeds.rightMetersPerSecond;


        // SmartDashboard.putNumber("path Output left", leftOutput);
        // SmartDashboard.putNumber("path Output right", rightOutput);
// ;
//         SmartDashboard.putNumber("current path time", trajTimer.get());

    }

}