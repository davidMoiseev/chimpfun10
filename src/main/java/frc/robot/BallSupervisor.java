package frc.robot;
import org.hotutilites.hotInterfaces.IHotSensedActuator;
import org.hotutilites.hotcontroller.HotController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.conveyor.feedModes;
import frc.robot.shooter.HoodPosition;

public class BallSupervisor implements IHotSensedActuator <RobotState, RobotCommandProvider, Double> {
    shooter shooter = new shooter();
    intake intake = new intake();
    conveyor conveyor = new conveyor();
    private HotController operator;
    private RobotState robotState;
    private boolean alreadyShooting;

    public BallSupervisor(final RobotState robotState) {
        this.robotState = robotState;

    }
    
    @Override
    public void setRobotState(final RobotState robotState) {
        this.robotState = robotState;
    }

    @Override
    public void updateState() {
        shooter.read();
        shooter.Display();
        conveyor.display();
        if(robotState.isRobotEnabled() == false){
            SmartDashboard.putNumber("MasterIndicator",0);
            robotState.setLEDColorState(4);
            robotState.setLEDFlash(false);  
        }else if(shooter.isInFault() || conveyor.inWarning() || conveyor.inCritical() && robotState.getVisionOutputStatus() == 0){
            this.robotState.setFault(true);
            SmartDashboard.putNumber("MasterIndicator",0);
            robotState.setLEDColorState(0);
        }else if(conveyor.getStatusLightState() == 3 && shooter.isShooterStable() && robotState.getVisionOutputStatus() == 3){
            SmartDashboard.putNumber("MasterIndicator", 3);
            robotState.setLEDColorState(3);
        }else if(conveyor.getStatusLightState() == 3 && shooter.PIDTarget != 0 && robotState.getVisionOutputStatus() == 2){
            SmartDashboard.putNumber("MasterIndicator", 2);
            robotState.setLEDColorState(2);
        }else{
            SmartDashboard.putNumber("MasterIndicator",1);
            robotState.setLEDColorState(1);
        }
        this.robotState.setInventory(conveyor.BallStored());
        SmartDashboard.putNumber("BallInventory", conveyor.BallStored());
    }

    @Override
    public void zeroSensor() {
        conveyor.stage(feedModes.reset);
        //conveyor.stage(feedModes.confirm);  
    }

    @Override
    public void setSensorValue(final Double value) {
        // TODO Auto-generated method stub

    }

    @Override
    public void performAction(final RobotCommandProvider commander, final RobotState robotState) {
        shooter.Display();
        conveyor.display();
        conveyor.updateStatusLight();
        shooter.updateStatus();
        operator = commander.getOperator();
        switch(commander.getBallSupervisorState()){
            case shootNsuck:
                robotState.setLEDColorState(3);
                robotState.setLEDFlash(true);
                if (shooter.isShooterStable() == true || alreadyShooting) {
                    conveyor.stage(feedModes.shoot);
                    shooter.indexPower(Calibrations.ballSuperviserVals.indexerPower);
                    alreadyShooting = true;
                }else{
                    conveyor.stage(feedModes.prime);
                    shooter.indexPower(0);
                }
                shooter.PIDmotor(robotState.getShooterTargetRPM());
                conveyor.setIntakeOn(true);
                intake.consume(Calibrations.ballSuperviserVals.intakeStandardPower);
            break;
            case intakeIn: //Once we are commanded by teleopcommander to run the intake we execute this case which powers the intake
                SmartDashboard.putString("BallState", "Intake in");
                conveyor.stage(feedModes.autoFill);
                shooter.stopPIDMotor();
                shooter.indexPower(0);
                if(true){
                    conveyor.setIntakeOn(true);
                    if(conveyor.reverseTime_1 > 0 || conveyor.conveyorBounceBack){
                        intake.consume(0);
                    }else{
                        intake.consume(Calibrations.ballSuperviserVals.intakeStandardPower);
                    }
                }
            break;
            case intakeOut:// when commanded to have intake reverse we command a negative power on the intake
                SmartDashboard.putString("BallState", "Intake out");
                conveyor.stage(feedModes.autoFill);
                shooter.stopPIDMotor();
                shooter.indexPower(0);
                intake.consume(-Calibrations.ballSuperviserVals.intakeStandardPower);
                conveyor.setIntakeOn(true);
            break;
            case intakeStop: //Set to stop the intake
                alreadyShooting = false;
                SmartDashboard.putString("BallState", "Intake stop");
                conveyor.stage(feedModes.autoFill);
                shooter.stopPIDMotor();
                shooter.indexPower(0);
                intake.consume(0);
                robotState.setLEDFlash(false);
                conveyor.setIntakeOn(false);
            break;
            case prime: //when we need to prime the robot for shooting we start spooling up the shooter to the requested
                //Rpm and we also begin to pusu balls forward so we have a ball instantly ready
                SmartDashboard.putString("BallState", "prime");
                conveyor.stage(feedModes.prime);
                shooter.PIDmotor(robotState.getShooterTargetRPM());
                shooter.indexPower(0);
                intake.consume(0);
                conveyor.setIntakeOn(false);
                shooter.setHood(HoodPosition.trench);
            break;
            case reject:
                //this is how we get balls out of the conveyor system
                SmartDashboard.putString("BallState", "reject");
                conveyor.stage(feedModes.reject);
                shooter.stopPIDMotor();
                shooter.indexPower(-Calibrations.ballSuperviserVals.indexerPower);
                intake.consume(-Calibrations.ballSuperviserVals.intakeStandardPower);
                conveyor.setIntakeOn(false);
            break;
            case shoot: //here we start feeding balls forward into the shooter
                SmartDashboard.putString("BallState", "shoot");
                robotState.setLEDColorState(3);
                robotState.setLEDFlash(true);
                if (shooter.isShooterStable() == true || alreadyShooting) {
                    conveyor.stage(feedModes.shoot);
                    shooter.indexPower(Calibrations.ballSuperviserVals.indexerPower);
                    alreadyShooting = true;
                }else{
                    conveyor.stage(feedModes.prime);
                    shooter.indexPower(0);
                }
                shooter.PIDmotor(robotState.getShooterTargetRPM());
                conveyor.setIntakeOn(false);
                
            break;
            case sort:
                //normal idle state of the shooter
                SmartDashboard.putString("BallState", "sort");
                conveyor.stage(feedModes.autoFill);
                shooter.stopPIDMotor();
                shooter.indexPower(0);
                conveyor.setIntakeOn(false);
            break;
            case stop:
                SmartDashboard.putString("BallState", "stop");
                conveyor.stage(feedModes.stop);
                shooter.stopPIDMotor();
                shooter.indexPower(0);
                conveyor.setIntakeOn(false);
            break;
            case confirm:
                SmartDashboard.putString("BallState", "Confirm");
                conveyor.stage(feedModes.confirm);
                shooter.stopPIDMotor();
                shooter.indexPower(0);
                conveyor.setIntakeOn(false);
            break;
            case reset:
                conveyor.stage(feedModes.reset);
            break;
            case manual:
                robotState.setLEDColorState(2);
                robotState.setLEDFlash(true);
                double convey = 0, carousel = 0, intakin = 0;
                shooter.indexPower(operator.getLeftTrigger());
                shooter.manual(operator.getRightTrigger());
                if(operator.getButtonY()) convey = 0.6;
                if(operator.getButtonA()) convey = -0.6;
                if(operator.getButtonX()) carousel = 0.6;
                if(operator.getButtonB()) carousel = -0.6;
                if(operator.getButtonLeftBumper()) intakin = 0.7;
                if(operator.getButtonRightBumper()) intakin = -0.7;
                intake.consume(intakin);
                conveyor.setManualInputs(convey, carousel);
                conveyor.stage(feedModes.manual);
            break;   
        }    
        //Don't forget to comment this out cause davids stuff no work 
		
        switch(commander.getHoodPosition()){
            case autoshot:
                shooter.setHood(HoodPosition.autoshot);
            break;
            case trench:
                shooter.setHood(HoodPosition.trench);
            break;
            case wallShot:
                shooter.setHood(HoodPosition.wallShot);
            break;
            case goingUnder:
                shooter.setHood(HoodPosition.goingUnder);
            break;
        }
    
    }

    public enum hoodPos{
        goingUnder,
        trench,
        wallShot,
        autoshot
    }

    public enum BallSupervisorState {
        shootNsuck,
        intakeIn,
        intakeOut,
        intakeStop,
        prime,
        shoot,
        stop,
        sort,
        reject,
        confirm,
        reset,
        manual
    }

	public enum intakePosition {
        packaged,
        ground,
        limelite,
        middle
	}

    
}