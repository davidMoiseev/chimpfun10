/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
  
    
    //private SendableChooser autoChooser;
   
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
    
    double distance1 = 2.1;// 2.47; //2.4;
    double distance2 = 3;//3.15 //4.75
    double distance2Overshoot = 0.2;
    private double shotAngle = 0;
    private Timer timer;
    private Timer autoTimer;
	public double driveYawCorrection = 0;
	public boolean driveYawCorrectionEnabled = false;
    private boolean timedOutCase1 = false;
    private boolean trench;
    private double distanceJustShoot = 0.5;


        //1.7 m = wof
 

    public AutoRoutineRunner(RobotState robotState){ 

        //autoChooser = new SendableChooser();
        trench = Calibrations.auton.trench;
        this.robotState = robotState;
        driveStep = -1;
        driveYawCorrection = 0;
        driveYawCorrectionEnabled = false;
        autoTimer = new Timer();
        shotAngle = 0;
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
        SmartDashboard.putBoolean("111 timed out case 1", timedOutCase1);
        HotLogger.Log("auto_step_drive", driveStep);
        HotLogger.Log("rightDistance", robotState.getDriveDistanceRight());
        HotLogger.Log("leftDistance", robotState.getDriveDistanceLeft());
        HotLogger.Log("autoTime", autoTimer.get()); 


        if(trench){
        switch(driveStep){
            case -1:
                autoTimer.start();
                initConveyer = true;
                timer = new Timer();
                timer.start();
                driveStep++; //so init code can run
                break;
            case 0:
                initConveyer = false;
                primeAutoShot = true;
                 autoAiming = true;
            if((robotState.isReadyToShoot() && (robotState.getVisionOutputStatus() == 3)) || timer.get() > 2.8){
                driveStep++;
            }
            break;
            case 1:
                shooting = true;
                autoAiming = false;
                primeAutoShot = false;
                if((timer.get() > 3.5)){
                    timedOutCase1 = true;}

            if(robotState.getInventory() == 0 || timedOutCase1){ 
                inventoryEmptyFor++;  
            }
            if(inventoryEmptyFor >= 10){
                shotAngle = robotState.getTheta();
                if(timedOutCase1)
                {ballReset = true;}
                resetEncoders = true;
                autoAiming = false;
                driveStep++;
                intake = true;
            }
            break;

            case 2:
                autoAiming = false;
                shooting = false;
                resetEncoders = false;
                driveOutput = 0.2;
                intake = true;
                driveYawCorrection = shotAngle;
                driveYawCorrectionEnabled = true;

            if ((robotState.getDriveDistanceRight() >= distance1)){
                intake = true;
                driveStep++;
                }
            break;  

            case 3:
                intake = true;
                driveOutput = 0;
                turnOutput = 0.18;
                driveYawCorrectionEnabled = false;
                if(robotState.getTheta() > 0){
                    intake = true;
                    resetEncoders = true;
                    driveStep++;
                    // if(Timer.getMatchTime() < 5){
                    //     distance2 = 1.47;
                    // }
                    timer.reset();
                    timer.start();
                }
                break;
    
            case 4: 
    
                resetEncoders = false;
                turnOutput = 0;
                driveOutput = 0.32;
                intake = true;
                driveYawCorrection = 0;
                driveYawCorrectionEnabled = true;
                if ((robotState.getDriveDistanceRight() >= (distance2 - 0.31)) || (timer.get() > 4.5)){
                   
                    driveStep++;
                    intake = true;
                    }
                break;
            
            case 5:
                resetEncoders = false;
                turnOutput = 0;
                driveOutput = 0.15;
                driveYawCorrection = 0;
                driveYawCorrectionEnabled = true;
                intake = true;
            if ((robotState.getDriveDistanceRight() >= (distance2 - 0.15)) || (timer.get() > 4.5)){
              
                driveStep++;
                turnOutput = 0;
                intake = true;
                timer.reset();
                timer.start();
                }
            break;
            case 6:
             
                resetEncoders = false;
                turnOutput = 0;
                driveOutput = 0.05;
                driveYawCorrection = 0;
                driveYawCorrectionEnabled = true;
                if ((robotState.getDriveDistanceRight() >= distance2) || (timer.get() > 4.5)){
                    timer.stop();
                    timer.reset();
                    resetEncoders = true;
                    driveStep++;
           }
           break;
        case 7:
                resetEncoders = false;
                driveOutput = -0.65;
                turnOutput = 0;
                driveYawCorrection = 0;
                driveYawCorrectionEnabled = true;
                intake = true;
                if ((robotState.getDriveDistanceRight() <= -distance2 + (1 - distance2Overshoot))){
                    driveStep++;
                    timer.reset();
                    timer.start();
                    }
                break;
        case 8:
                intake = true;
                driveOutput = 0;
                turnOutput = -0.18;
                driveYawCorrectionEnabled = false;
                if(robotState.getTheta() < shotAngle || timer.get() > 4){
                    resetEncoders = true;
                    driveStep++;
                    // if(Timer.getMatchTime() < 5){
                    //     distance2 = 1.47;
                    // }
                   timer.stop();
                   timer.reset();
                   timer.start();
                }
                break;
                    
        case 9: 
                if (timer.get() > .075) {
                    resetEncoders = true;
                    driveStep++;
                
                    timer.stop();
                    timer.reset();
                    timer.start();
                       
                }
        break;
        case 10:
                resetEncoders = false;
                turnOutput = 0;
                driveOutput = -0.3;
                driveYawCorrection = shotAngle;
                driveYawCorrectionEnabled = true;
                primeAutoShot = true;
                autoAiming = false;
        
                if (robotState.getDriveDistanceRight() <= -distance1 + distance2Overshoot){
                    primeAutoShot = true;
                    autoAiming = false;
                    driveStep++;
                    }
                    break;  
         case 11:
                    driveOutput = 0.0;
                    primeAutoShot = true;
                    driveYawCorrectionEnabled = false;
                    autoAiming = true;
                if (autoTimer.get() > 13.75 ||  (robotState.isReadyToShoot() && (robotState.getVisionOutputStatus() == 3))){
                    driveStep++;
                }
                break;
            case 12:
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

        else{
            switch(driveStep){
                case -1:
                    autoTimer.start();
                    initConveyer = true;
                    timer = new Timer();
                    timer.start();
                    driveStep++; //so init code can run
                    break;
                case 0:
                    initConveyer = false;
                    primeAutoShot = true;
                     autoAiming = true;
                if(robotState.isReadyToShoot() && (robotState.getVisionOutputStatus() == 3) || (timer.get() > 3)){
                    driveStep++;
                }
                break;
                case 1:
                    shooting = true;
                    autoAiming = false;
                    primeAutoShot = false;
                    if((timer.get() > 7)){
                        timedOutCase1 = true;}
    
                if(robotState.getInventory() == 0 ){ 
                    inventoryEmptyFor++;  
                }
                if(inventoryEmptyFor >= 10 || timedOutCase1){
                    shotAngle = robotState.getTheta();
                    if(timedOutCase1)
                    {ballReset = true;}
                    resetEncoders = true;
                    autoAiming = false;
                    driveStep++;
                }
                break;
    
                case 2:
                    autoAiming = false;
                    shooting = false;
                    resetEncoders = false;
                    driveOutput = -0.15;
                    intake = false;
                    driveYawCorrection = shotAngle;
                    driveYawCorrectionEnabled = true;
    
                if (robotState.getDriveDistanceRight() <= -distanceJustShoot){
                    intake = false;
                    driveStep++;
                    }
                break;  
                
                case 3:
                    driveOutput = 0;
                    break;
        }
    }
            
    }
}

