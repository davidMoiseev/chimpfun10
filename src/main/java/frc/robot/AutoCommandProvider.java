/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;



import frc.robot.TrajectoryFollower.PathNames;

import org.hotutilites.hotcontroller.HotController;
import frc.robot.BallSupervisor.BallSupervisorState;
import frc.robot.BallSupervisor.hoodPos;
import frc.robot.Arm.ArmPositions;
import frc.robot.AutoRoutineRunner;

public class AutoCommandProvider extends RobotCommandProvider {

    private AutoRoutineRunner autoRunner;
    private RobotState robotState;

    public AutoCommandProvider(RobotState robotState) {
        this.robotState = robotState;

        autoRunner = new AutoRoutineRunner(robotState);

    }

    public void updateAutoRoutine() {
        autoRunner.update();
    }

    @Override
    public double getDriveCommand() {
        return autoRunner.driveOutput;
    }

    @Override
    public double getTurnCommand() {
        return autoRunner.turnOutput;
    }

    @Override
    public boolean getPathFollowingCommand() {
        return autoRunner.isPathFollowing;
    }

    @Override
    public PathNames getPathName() {
        return autoRunner.pathName;
    }

    @Override
    public double getArmOutput() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public boolean getAimingEnabled() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getRangeEnabled() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void setManualMode() {
        // TODO Auto-generated method stub

    }

    @Override
    public HotController getOperator() {
        // TODO Auto-generated method stub
        return null;
    }

    public void setConveyerAutoInit(){
        setBallSupervisorState(BallSupervisorState.confirm);
    }


    @Override
    public void chooseBallCommand() {
        
        

        if (autoRunner.shooting) {
            setBallSupervisorState(BallSupervisorState.shoot);
        } 

        else if (autoRunner.primeAutoShot) { // config for autoshot
            setBallSupervisorState(BallSupervisorState.prime);
            robotState.setShooterTargetRPM(3800);
            setHoodPosition(hoodPos.autoshot);
            setArmPosition(ArmPositions.autoshot);
        } 
        else if (autoRunner.primeTrenchShot) {// config for trench shot
            setBallSupervisorState(BallSupervisorState.prime);
            robotState.setShooterTargetRPM(5300);
            setHoodPosition(hoodPos.trench);
            setArmPosition(ArmPositions.trenchshot);
        } 
        else if (autoRunner.primeWallshot) { // Prime for wallshot
            setBallSupervisorState(BallSupervisorState.prime);
            robotState.setShooterTargetRPM(2400);
            setHoodPosition(hoodPos.wallShot);
            setArmPosition(ArmPositions.wallshot);
        } 

        else if (autoRunner.intake) {
            setBallSupervisorState(BallSupervisorState.intakeIn);
            setArmPosition(ArmPositions.ground);
        } 
        
        else if (autoRunner.ballReset) {
            setBallSupervisorState(BallSupervisorState.reset);
        } 
        
        else {
            // setBallSupervisorState(BallSupervisorState.intakeStop);
            // setHoodPosition(hoodPos.goingUnder);
            // robotState.setShooterTargetRPM(0);
            // setArmPosition(ArmPositions.ground);
          
        }
    }
    
}
