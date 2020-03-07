package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANPIDController.AccelStrategy;
import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import org.hotutilites.hotInterfaces.IHotSensedActuator;
import org.hotutilites.hotlogger.HotLogger;
public class Arm implements IHotSensedActuator<RobotState, RobotCommandProvider, Integer > {
    RobotState state;
    private TalonSRX armMotor;
    public boolean hasReset = false;
    private double currentDegreesFromHor;
    private double cosineScalar;
    private double currentCurrent;
    private int counter = 0;
    private double output;
    private boolean resetting;
    private boolean armReset = false;;
    public Arm(RobotState state){
        this.state = state;
        armMotor = new TalonSRX(Calibrations.CAN_ID.intakeLifter);
        armMotor.configFactoryDefault();
        
        
        armMotor.setInverted(false);
        armMotor.setSensorPhase(true);
        armMotor.setNeutralMode(NeutralMode.Brake);
        // armMotor.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Absolute, 20);
        armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 40);
    
     
        armMotor.configPeakCurrentLimit(45, 20); //Limit of 45 amps
        armMotor.configPeakCurrentDuration(750, 20); //For 500ms
        armMotor.configContinuousCurrentLimit(30, 20); //Limit of 30 amps cont
        armMotor.configNominalOutputForward(0, 0);
        armMotor.configNominalOutputReverse(0, 0);
        armMotor.configPeakOutputForward(1, 0);
        armMotor.configPeakOutputReverse(-1, 0);
        //config motion magic
    //distance profile and primarily closed
        armMotor.configNeutralDeadband(0.1, 30);
        armMotor.selectProfileSlot(0, 0); 
        armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 40);
        armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 40);
        armMotor.configMotionAcceleration(Calibrations.ARM.acc, 30);                
        armMotor.configMotionCruiseVelocity(Calibrations.ARM.vel, 30);
        armMotor.config_kP(0, Calibrations.ARM.kP, 30);             //0 = parameter slot, 0 is distance mode
        armMotor.config_kI(0, Calibrations.ARM.kI, 30);
        armMotor.config_kD(0, Calibrations.ARM.kD, 30);
        armMotor.config_kF(0, Calibrations.ARM.kF, 30);
        armMotor.config_IntegralZone(0, Calibrations.ARM.kIZ);
        this.zeroSensor();
    }
    public enum ArmPositions{      
        packaged,
        wallshot,
        autoshot,
        trenchshot,
        ground,
        manual,
        wheelOfFortune,
        off,
        reset
    }
    public enum ArmStates{            
        changingPosition,
        atPosition,
        resetting,
        freefall                        //nothing commanded, gravity controlled. (should be default so robot doesn't drop when disabled, safety risk, position button held at all time?)
    }
    public void performAction(RobotCommandProvider commander, RobotState state){  
      
  
        SmartDashboard.putNumber("arm motion magc error", armMotor.getClosedLoopError(0));
        resetting = false; //set here so will default to set to false unless runs case that is resetting


        if(armMotor.getSelectedSensorPosition() > Calibrations.ArmPositions.maxTicks){
            armMotor.set(ControlMode.PercentOutput, 0);
        }
        else{
        switch(commander.getArmPosition()){
            case packaged:
                armMotor.set(ControlMode.MotionMagic, Calibrations.ArmPositions.packagedAngle*Calibrations.ARM.ticksPerDegree);
                // armMotor.set(ControlMode.MotionMagic, Calibrations.ARM.packagedAngleSetPoint*Calibrations.ARM.ticksPerDegree, DemandType.ArbitraryFeedForward, calcArbFF());
            break;                                                                  //ticks per degree prob 360/4096 * gear ratio?
            case wallshot:
                armMotor.set(ControlMode.MotionMagic,  Calibrations.ArmPositions.wallShotAngle*Calibrations.ARM.ticksPerDegree);
            
              // armMotor.set(ControlMode.MotionMagic, Calibrations.ARM.visionAngleSetPoint*Calibrations.ARM.ticksPerDegree, DemandType.ArbitraryFeedForward, calcArbFF()); 
            break;
            case autoshot:
                armMotor.set(ControlMode.MotionMagic,  Calibrations.ArmPositions.autoShotAngle*Calibrations.ARM.ticksPerDegree);
            
              // armMotor.set(ControlMode.MotionMagic, Calibrations.ARM.visionAngleSetP1    oint*Calibrations.ARM.ticksPerDegree, DemandType.ArbitraryFeedForward, calcArbFF()); 
            break;
            case trenchshot:
                armMotor.set(ControlMode.MotionMagic,  Calibrations.ArmPositions.trenchShotAngle*Calibrations.ARM.ticksPerDegree);
            
            break;
            case ground:
                if(!armReset){
                    if(hardResetComplete()){
                        resetting = false;
                        armReset = true;
                    }
                    else{
                        resetting = false;
                    }
                }
                else{
                  armMotor.set(ControlMode.MotionMagic, Calibrations.ArmPositions.groundPickupAngle*Calibrations.ARM.ticksPerDegree);
               
                }
               break;
            case manual:
                armMotor.set(ControlMode.PercentOutput, commander.getArmOutput());
                armReset = false;
            break;
            case wheelOfFortune:
                armMotor.set(ControlMode.MotionMagic, Calibrations.ArmPositions.wofAngle*Calibrations.ARM.ticksPerDegree);
            break;
            case off:
                armMotor.set(ControlMode.PercentOutput, 0);
            break;
            case reset:
                armMotor.set(ControlMode.PercentOutput, 0.00);  
                armMotor.setSelectedSensorPosition(0, 0, 0);
                resetting = true;
          
            break;
            }
        }   
        }
     
    public void setArmReset(boolean reset){
        if(reset){
            armReset = false;
        }else{
            armReset = true;
        }
    }

    public boolean hardResetComplete(){
        if  (currentCurrent > Calibrations.ARM.resetCurrentDraw){    
            armMotor.set(ControlMode.PercentOutput, 0.00);  
            armMotor.setSelectedSensorPosition(0);
            counter = 0;
            return true;
        }
        else {
           armMotor.set(ControlMode.PercentOutput, -0.3);     //need to determine arm motor output direction
            counter++;
            resetting = true;
            return false;
        }
        
    }
    private double calcArbFF(){
        currentDegreesFromHor = (armMotor.getSelectedSensorPosition()/ Calibrations.ARM.ticksPerDegree);
        cosineScalar = java.lang.Math.cos(Math.toRadians(currentDegreesFromHor));
        return Calibrations.ARM.maxGravityFF * cosineScalar;     //maxGravFF = output to apply when arm is horizontal
    }
    
    @Override
    public void updateState() {
        currentCurrent = Math.abs(armMotor.getStatorCurrent());
        HotLogger.Log("has arm reset", armReset );
        HotLogger.Log("arm commanded", armMotor.getMotorOutputPercent());
        HotLogger.Log("arm current", armMotor.getStatorCurrent());
        SmartDashboard.putNumber("current current", currentCurrent);
        if(resetting = true){
            state.setArmState(ArmStates.resetting);
        }
        if(armMotor.getClosedLoopError() < 20){
            state.setArmState(ArmStates.atPosition);
        }
        else{
            state.setArmState(ArmStates.changingPosition);
        }
        state.setArmAngleDegreesFrom90((armMotor.getSelectedSensorPosition() - Calibrations.ARM.ticksAt90) / Calibrations.ARM.ticksPerDegree);
    }
    @Override
    public void zeroSensor() {
        hasReset = false;
        armMotor.set(ControlMode.PercentOutput, 0.00);  
        armMotor.setSelectedSensorPosition(0, 0, 0);
    }

    public void autoInitArmAngle(){
        armMotor.set(ControlMode.PercentOutput, 0.00);  
        armMotor.setSelectedSensorPosition(7700, 0, 0); 
    }

    @Override
    public void setSensorValue(Integer value) {
        // TODO Auto-generated method stub
    }
    @Override
    public void setRobotState(RobotState robotState) {
        // TODO Auto-generated method stub
    }
  
}  