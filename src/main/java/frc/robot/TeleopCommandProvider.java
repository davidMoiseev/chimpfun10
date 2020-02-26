package frc.robot;

import org.hotutilites.hotcontroller.HotController;
import frc.robot.BallSupervisor.BallSupervisorState;
import frc.robot.BallSupervisor.hoodPos;
import frc.robot.Arm.ArmPositions;

public class TeleopCommandProvider extends RobotCommandProvider {
    private HotController driver;
    private HotController operator;
    private RobotState robotState;
    private boolean setModeEdge;
    private boolean setModeEdge2;
    private boolean manaulMode;
    private boolean lowPowerMode;
    private double turn;

    public TeleopCommandProvider(HotController driver, HotController operator, RobotState robotState) {
        this.setDriver(driver);
        this.setOperator(operator);
        this.robotState = robotState;
    }

    public boolean isLowPowerMode() {
        return lowPowerMode;
    }
    
    public void setLowPowerMode() {
        if(driver.getButtonBack() != setModeEdge2){
            if(driver.getButtonBack()){
                lowPowerMode = !lowPowerMode;
            }
        }
        setModeEdge2 = driver.getButtonBack();
    }

    public HotController getOperator() {
        return operator;
    }
    public void setOperator(HotController operator) {
        this.operator = operator;
    }
    public double getDriveCommand() {
        return driver.getStickLY();
    }
    public double getTurnCommand() {
        turn = driver.getStickRX();
        if(turn < 0){
            return -Math.pow(Math.abs(turn), 2);
        }
        else{
            return Math.pow(turn, 2);
        }
    }
    public double getArmOutput(){    //for testing, normally disable
        return -operator.getStickLY();
    }
    public boolean getAimingEnabled(){
        return driver.getButtonA();
    }
    public boolean getRangeEnabled(){
        return driver.getButtonX();
    }
    public boolean getManualMode(){
        return manaulMode;
    }
   
    public void lockManualMode(boolean mode){
        manaulMode = mode;
    }

    public void rezeroArm(){
        if(operator.getButtonBack()){
            setArmPosition(ArmPositions.reset);
        }
    }
    private void setDriver(HotController driver) {
        this.driver = driver;
    }
    public void setManualMode(){
        if(operator.getButtonBack() != setModeEdge){
            if(operator.getButtonBack()){
                manaulMode = !manaulMode;
            }
        }
        setModeEdge = operator.getButtonBack();
    }
    

    @Override
    public void chooseBallCommand() {
        if(this.getManualMode()){
            setBallSupervisorState(BallSupervisorState.manual);
            setArmPosition(ArmPositions.manual);
        }else if(this.isLowPowerMode()){
            setBallSupervisorState(BallSupervisorState.stop);
        }else if(driver.getRightTrigger() > 0.5 && operator.getButtonLeftBumper()){
            setBallSupervisorState(BallSupervisorState.shootNsuck);
        }else if(driver.getRightTrigger() > 0.5){
            setBallSupervisorState(BallSupervisorState.shoot);
        }else if (operator.getButtonX()){ //config for autoshot
            setBallSupervisorState(BallSupervisorState.prime);
            robotState.setShooterTargetRPM(3600);
            setHoodPosition(hoodPos.autoshot);
            setArmPosition(ArmPositions.autoshot);
        }else if(operator.getButtonB()){//config for trench shot
            setBallSupervisorState(BallSupervisorState.prime);
            robotState.setShooterTargetRPM(5200);
            setHoodPosition(hoodPos.trench);
            setArmPosition(ArmPositions.trenchshot);
        }else if(operator.getButtonA()){ //Prime for wallshot
            setBallSupervisorState(BallSupervisorState.prime);
            robotState.setShooterTargetRPM(2800);
            setHoodPosition(hoodPos.wallShot);
            setArmPosition(ArmPositions.wallshot);
        }else if(operator.getButtonY()){
            setBallSupervisorState(BallSupervisorState.reject);
        }else if(operator.getButtonLeftBumper()){
            setBallSupervisorState(BallSupervisorState.intakeIn); 
            setArmPosition(ArmPositions.ground);
        }else if(operator.getButtonRightBumper()){
            setBallSupervisorState(BallSupervisorState.intakeOut); 
        }else if(operator.getButtonRightStick()){
            setBallSupervisorState(BallSupervisorState.reset);
        }else if(operator.getButtonStart()){
            setBallSupervisorState(BallSupervisorState.confirm);
        }else{
            setBallSupervisorState(BallSupervisorState.intakeStop); 
            setHoodPosition(hoodPos.goingUnder);
            robotState.setShooterTargetRPM(0);
            setArmPosition(ArmPositions.ground);
        }
    }
        
}