package frc.robot;

import org.hotutilites.hotcontroller.HotController;
import frc.robot.BallSupervisor.BallSupervisorState;
import frc.robot.BallSupervisor.hoodPos;
import frc.robot.TrajectoryFollower.PathNames;
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
    private double operatorLX = 0;

    public TeleopCommandProvider(HotController driver, HotController operator, RobotState robotState) {
        this.setDriver(driver);
        this.setOperator(operator);
        this.robotState = robotState;
    }


    public boolean isLeftClimberActivate() {
        return (operator.getLeftTrigger() > 0.5 && operator.getRightTrigger() > 0.5 && !manaulMode) || (operator.getButtonLeftStick() && manaulMode) ;
    }

    public boolean isRightClimberActivate() {
        return  this.isLeftClimberActivate();
    }

    public double getLeftClimberDelta(){
        if(manaulMode){
            return operator.getStickLY();
        }else{
            if(Math.abs(operator.getStickLX()) < 0.02 ){
                operatorLX = 0;
            }else{
                operatorLX = ((1.02 * operator.getStickLX()) - 0.02);
            }return operator.getStickLY() + operatorLX;
        }
    }
    public double getRightClimberDelta(){
        if(manaulMode){
            return operator.getStickRY();
        }else{
            if(Math.abs(operator.getStickLX()) < 0.02 ){
                operatorLX = 0;
            }else{
                operatorLX = ((1.02 * operator.getStickLX()) - 0.02);
            }return operator.getStickLY() - operatorLX;
        }
        
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
        return -driver.getStickLY();
    }
    public double getTurnCommand() {
        turn = driver.getStickRX();
        if (turn < 0) {
            return -Math.pow(Math.abs(turn), 2);
        } else {
            return Math.pow(turn, 2);
        }
    }
    public double getArmOutput(){    //for testing, normally disable
        if(this.isLeftClimberActivate()){
            return 0;
        }else{
            return -operator.getStickLY();
        }
    }
    public boolean getAimingEnabled(){
        return driver.getButtonA();
    }

    public boolean getRangeEnabled() {
        return driver.getButtonX();
    }
    public boolean getManualMode() {
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
        if (this.getManualMode()) {
            setBallSupervisorState(BallSupervisorState.manual);
            setArmPosition(ArmPositions.manual);
        }else if(driver.getRightTrigger() > 0.5 && operator.getButtonStart()){
            setBallSupervisorState(BallSupervisorState.shootNstuck);
        }else if(driver.getRightTrigger() > 0.5){
            setBallSupervisorState(BallSupervisorState.shoot);
        }else if (operator.getButtonX()){ //config for autoshot
            setBallSupervisorState(BallSupervisorState.prime);
            robotState.setShooterTargetRPM(3040); //to const  //3800
            setHoodPosition(hoodPos.autoshot);
            setArmPosition(ArmPositions.autoshot);
            
        }else if(operator.getButtonB()){//config for trench shot
            setBallSupervisorState(BallSupervisorState.prime);
            robotState.setShooterTargetRPM(5400); //to const //5300
            setHoodPosition(hoodPos.trench);
            setArmPosition(ArmPositions.trenchshot);   

        }else if(operator.getButtonY()){
            setBallSupervisorState(BallSupervisorState.prime);
            robotState.setShooterTargetRPM(2500); //to const
            setHoodPosition(hoodPos.oneBotBack);
            setArmPosition(ArmPositions.wallshot);

        }else if(operator.getButtonA()){ //Prime for wallshot
            setBallSupervisorState(BallSupervisorState.prime);
            robotState.setShooterTargetRPM(2800); //to const
            setHoodPosition(hoodPos.wallShot);
            setArmPosition(ArmPositions.wallshot);

        }else if(operator.getDpad() == 180){
            setBallSupervisorState(BallSupervisorState.prime);
            robotState.setShooterTargetRPM(1500); //to const //5300
            setHoodPosition(hoodPos.wallShot); 

        }else if(operator.getButtonStart()){
            setBallSupervisorState(BallSupervisorState.reject);
        }else if(operator.getButtonLeftBumper()){
            setBallSupervisorState(BallSupervisorState.intakeIn); 
            setArmPosition(ArmPositions.ground);
        }else if(operator.getButtonRightBumper()){
            setBallSupervisorState(BallSupervisorState.intakeOut); 
        }else if(operator.getButtonRightStick()){
            setBallSupervisorState(BallSupervisorState.reset);
        }else if(operator.getButtonLeftStick()){
            setArmPosition(ArmPositions.wheelOfFortune);
            setBallSupervisorState(BallSupervisorState.intakeStop);
        
        }else{
            setBallSupervisorState(BallSupervisorState.intakeStop); 
            setHoodPosition(hoodPos.goingUnder);
            robotState.setShooterTargetRPM(0);
            setArmPosition(ArmPositions.off);
            robotState.setTurnOnLimeLiteLight(false);
        }
    }

    @Override
    public PathNames getPathName() {
        
        return null;
    }

    @Override
    public boolean getPathFollowingCommand() {
        return false;
    }

    @Override
    public boolean getEncodersReset() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getDriveYawCorrectionEnabled() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public double getDriveYawCorrection() {
        // TODO Auto-generated method stub
        return 0;
    }

}


