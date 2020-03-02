/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autos.AutoRoutineBase;
import frc.robot.Autos.SimpleRoutine;
import frc.robot.TrajectoryFollower.PathNames;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class AutoRoutineRunner {

    public boolean isPathFollowing = false;
    public PathNames pathName;
    public double driveOutput = 0;
    public double turnOutput = 0;
    public autoModes currentDriveMode;
    public BallModes currentBallMode;
    private int driveStep;
    private int ballStep;
    private boolean isDriveStepFinished = false;
    private boolean isBallStepFinished = false;
    private int inventoryEmptyFor = 0;
    


    RobotState robotState;
	public boolean shooting = false;
	public boolean primeAutoShot = false;
	public boolean primeTrenchShot = false;
	public boolean primeWallshot = false;
	public boolean intake = false;
    public boolean ballReset = false;
    public boolean autoAiming = false;
    public boolean initConveyer = false;
    public boolean resetEncoders = false;
    
    double distance1 = 2.4; //2.4;
    double distance2 = 3.15;
    private double shotAngle = 0;
    private Timer timer;

 

    public AutoRoutineRunner(RobotState robotState){ 

        this.robotState = robotState;
        driveStep = -1;
        // ballStep = 1;

        // isDriveStepFinished = false;
        
   
    }

    public enum autoModes{
        followingShootingStartPosToWofFront,
        followingSamplePath2,
        shooting, 
        turningToFront,
        autoAlign,
        nothing
    }

    public enum BallModes{
        primingAutoShot,
        primingTrenchShot,
        primingWallShot,
        shooting, 
        intaking, 
    }

    public void update(){



        ///////////////drive logic ////////////////////

        // SmartDashboard.putNumber("drive step", driveStep);
        // SmartDashboard.putNumber("ball step", ballStep);
        // HotLogger.Log("auto_step_drive", driveStep);
        // HotLogger.Log("auto_step_ball", ballStep);
        // HotLogger.Log("isBallStepFinished", isBallStepFinished);
        // HotLogger.Log("isDriveStepFinished", isDriveStepFinished);

    //     switch(driveStep){
    //         case 0: //finished auton

    //             currentDriveMode = autoModes.nothing;
    //             turnOutput = 0;
    //             driveOutput = 0;
    //             isPathFollowing = false;
    //             autoAiming = false;
    //             break;

    //         case 1:
    //             currentDriveMode = SimpleRoutine.state1.driveMode;
    //             break;

    //         case 2:
    //             currentDriveMode = SimpleRoutine.state2.driveMode;
    //             break;
    //     }

    //     if(isDriveStepFinished && ((driveStep > 0) && (driveStep < SimpleRoutine.numberOfSteps))){
    //         isDriveStepFinished = false;
    //         driveStep++;
    //     }
    //     else if (isDriveStepFinished && (driveStep >= SimpleRoutine.numberOfSteps)){
    //         isDriveStepFinished = false;
    //         driveStep = 0;
    //     }

    //     /////////////ball logic //////////////////////

    //     switch(ballStep){
    //         case 0: //finished auton
    //             shooting = false;
    //             primeAutoShot = false;
    //             primeTrenchShot = false;
    //             primeWallshot = false;
    //             intake = false;
    //             ballReset = false;
    
    //             break;

    //         case 1:
    //             currentBallMode = SimpleRoutine.state1.ballMode;
    //             break;

    //         case 2:
    //             currentBallMode = SimpleRoutine.state2.ballMode;
    //             break;
    //     }

    //     if(isBallStepFinished && ((ballStep > 0) && (ballStep < SimpleRoutine.numberOfSteps))){
    //         isBallStepFinished = false;
    //         driveStep++;
    //     }
    //     else if (isBallStepFinished && (ballStep >= SimpleRoutine.numberOfSteps)){
    //         isBallStepFinished = false;
    //         driveStep = 0;
    //     }

    //     ///////////drive modes /////////////////////

    //     if(currentDriveMode == autoModes.turningToFront){    //put these in clumps of what is commanded so two variables can't be set at once

    //         turnOutput = 0.3;
    //         if(robotState.getTheta() > 178 && robotState.getTheta() < 182){
    //             isDriveStepFinished = true;
    //         }

    //         else {
    //             isDriveStepFinished = false;
    //         }
    //     }

    //     else if(currentDriveMode == autoModes.autoAlign){
    //         if(!(robotState.getVisionOutputStatus() == 3)){
    //             isDriveStepFinished = true;
    //             autoAiming = false;
    //         }
    //         else {
    //             isDriveStepFinished = false;
    //             autoAiming = true; 
    //         }
    //     }

    //     else{
    //         turnOutput = 0;
    //         autoAiming = false;
    //     }


    //     //paths

    //     if(currentDriveMode == autoModes.followingSamplePath){

    //         isPathFollowing = true;
    //         pathName = PathNames.sample;
           
    //         if(robotState.getTrajectoryComplete()){
    //             isDriveStepFinished = true;
    //             isPathFollowing = false;
    //         }
    //         else{
    //             isDriveStepFinished = false;
                
    //         }
    //     }
    //     else if(currentDriveMode == autoModes.followingSamplePath2){

    //         isPathFollowing = true;
    //         pathName = PathNames.sample2;
    //         if(robotState.getTrajectoryComplete()){
    //             isDriveStepFinished = true;
    //             isPathFollowing = false;
    //         }
    //         else{
    //             isDriveStepFinished = false;
                
    //         }
    //     }
    //     else{
    //         isPathFollowing = false;
    //     }

    //     /////////////// ball modes ///////////////////////

    //     if(currentBallMode == BallModes.primingAutoShot){


    //         if(robotState.isReadyToShoot()){

    //             isBallStepFinished = true;
    //             primeAutoShot = false;
    //         }
    //         else{
    //             primeAutoShot = true;
    //             isBallStepFinished = false;
    //         }

            
    //     }
      
    //     else if(currentBallMode == BallModes.shooting){

    //         if(robotState.getInventory() == 0){
    //             inventoryEmptyFor++;
    //             primeAutoShot = true;
    //             isBallStepFinished = false;
    //         }

    //         else if(inventoryEmptyFor >= 5){
    //             isBallStepFinished = true;
    //             primeAutoShot = false;
    //             ballReset = true;

    //         }
    //         else{
    //             inventoryEmptyFor = 0;
    //             primeAutoShot = true;
    //             isBallStepFinished = false;
    //         }
    //     }

    //     else {
    //         ballReset = false;
    //         primeAutoShot = false;
    //         inventoryEmptyFor = 0;
    //     }
    //  }
        SmartDashboard.putNumber("11drive step", driveStep);
        SmartDashboard.putBoolean("11 ready to shoot", robotState.isReadyToShoot());
        SmartDashboard.putNumber("11 vision ready", robotState.getVisionOutputStatus());
        SmartDashboard.putNumber("11 inventory empty for", inventoryEmptyFor);
        HotLogger.Log("auto_step_drive", driveStep);

        // switch(driveStep){
        //     case -1:
        //         isPathFollowing = true;
        //         pathName = PathNames.shootingStartPosToWofFront;

        //         if (robotState.getTrajectoryComplete()){
        //             driveStep++;
        //         }

        //         break;

        //     case 0:
        //         isPathFollowing = false;
        //         break;
                

        // }

        switch(driveStep){
            case -1:
                initConveyer = true;
                driveStep++; //so init code can run
                timer = new Timer();
                break;
            case 0:
                initConveyer = false;
                primeAutoShot = true;
                 autoAiming = true;
            if(robotState.isReadyToShoot() && (robotState.getVisionOutputStatus() == 3)){
                
                driveStep++;
            }
            break;
            case 1:
                shooting = true;
                autoAiming = false;
                primeAutoShot = false;

            if(robotState.getInventory() == 0){ 
                inventoryEmptyFor++;  
            }
            if(inventoryEmptyFor >= 10){
                shotAngle = robotState.getTheta();
                ballReset = true;
                resetEncoders = true;
                driveStep++;
            }
            break;

            case 2:
                shooting = false;
                 resetEncoders = false;
                driveOutput = 0.2;
                intake = true;
                SmartDashboard.putNumber("1111 inthingu drivedistance left", robotState.getDriveDistanceLeft());
                SmartDashboard.putNumber("1111 inthingy drive distance right", robotState.getDriveDistanceRight());

            if ((robotState.getDriveDistanceLeft() >= distance1) || (robotState.getDriveDistanceRight() >= distance1)){
                intake = true;
                driveStep++;
                }
            break;  

            case 3:
                intake = true;
                driveOutput = 0;
                turnOutput = 0.08;
                if(robotState.getTheta() > 0){
                intake = true;
                resetEncoders = true;
                driveStep++;
                timer.start();
                }
                break;

            case 4: 
                resetEncoders = false;
                turnOutput = 0;
                driveOutput = 0.2;
                intake = true;

                if ((robotState.getDriveDistanceLeft() >= distance2) || (robotState.getDriveDistanceRight() >= distance2) || (timer.get() > 4.5)){
                    driveStep++;
                    resetEncoders = true;
                    intake = true;
                    }
                break;

            case 5:
                resetEncoders = false;
                driveOutput = -0.2;
                intake = true;
                if ((robotState.getDriveDistanceLeft() <= -distance2) || (robotState.getDriveDistanceRight() <= -distance2)){
                    driveStep++;
                    }
                break;
            case 6:
                driveOutput = 0;
                turnOutput = -0.08;
                if(robotState.getTheta() < shotAngle){
                resetEncoders = true;
                driveStep++;
                }
                break;
            case 7:
                resetEncoders = false;
                turnOutput = 0;
                driveOutput = -0.2;
        
                if ((robotState.getDriveDistanceLeft() <= -distance1) || (robotState.getDriveDistanceRight() <= -distance1)){
                    driveStep++;
                    }
                    break;  
            case 8:
                    driveOutput = 0;
                    primeAutoShot = true;
                    autoAiming = true;
                if(robotState.isReadyToShoot() && (robotState.getVisionOutputStatus() == 3)){
                    driveStep++;
                }
                break;
            case 9:
                    shooting = true;
                    autoAiming = false;
                    primeAutoShot = false;

                if(robotState.getInventory() == 0){ 
                    inventoryEmptyFor++;  
                }
                if(inventoryEmptyFor >= 5){
                    ballReset = true;
                }
                break;
                
                
        }


            
    }
}

