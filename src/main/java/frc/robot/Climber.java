package frc.robot;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.hotutilites.hotInterfaces.IHotSensedActuator;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber implements IHotSensedActuator<RobotState, RobotCommandProvider, Integer> {
    RobotState robotState;
    climberController leftClimber;
    climberController rightClimber;
    Solenoid rachet;

    private class climberController {
        private TalonFX motor;
        private int statusColor;
        private boolean enabled;
        private boolean released;
        private boolean inAuto;
        private boolean override;
        private double overridePower;
        private double target;
        private double autoTarget;
        private double delta;
        private double position;
        private double SnapShot;
        private boolean tookSnapShot;

        public climberController(int canID) {
            motor = new TalonFX(canID);
            motor.configFactoryDefault();
            motor.selectProfileSlot(0, 0);
            motor.configNominalOutputForward(0);
            motor.configNominalOutputReverse(0);
            motor.configPeakOutputForward(1);
            motor.configPeakOutputReverse(-1);
            motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, canID);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);
            motor.configMotionAcceleration(Calibrations.climberCals.acc);
            motor.configMotionCruiseVelocity(Calibrations.climberCals.vel);
            motor.config_kP(0, Calibrations.climberCals.kP);
            motor.config_kI(0, Calibrations.climberCals.kI);
            motor.config_kD(0, Calibrations.climberCals.kD);
            motor.config_kF(0, Calibrations.climberCals.kF);
            motor.config_IntegralZone(0, Calibrations.climberCals.kIZ);
            motor.configForwardSoftLimitEnable(true);
            motor.configReverseSoftLimitEnable(true);
            motor.configForwardSoftLimitThreshold(
                    (int) (Calibrations.climberCals.upperLimit * Calibrations.climberCals.ticksPerInch));
            motor.configReverseSoftLimitThreshold(0);
        }

        public int getStatusColor() {
            return statusColor;
        }

        public void setPower(double power) {
            this.overridePower = power;
        }

        public void setEnabled(boolean enabled) {
            this.enabled = enabled;
        }

        public void setAuto(boolean mode) {
            inAuto = mode;
        }

        public void reset() {
            autoTarget = 0;
            statusColor = 0;
            delta = 0;
            SnapShot = 0;
            tookSnapShot = false;
            motor.setSelectedSensorPosition(0);
            inAuto = false;
            released = false;
            enabled = false;
        }

        public void setOverride(boolean mode){
            override = mode;
        }

        public void setTarget(double targetInch){
            autoTarget = (Calibrations.climberCals.ticksPerInch * targetInch);
        }

        public void setDelta(double deltaInch){
            delta = (Calibrations.climberCals.ticksPerInch * deltaInch);
        }

        private void release(){
            target = position + 4;
            if(position > SnapShot - 0.25 || position < 0.25){
                released = true;
            }else{
                released = false;
            }
        }

        public void periodic(){
            position = motor.getSelectedSensorPosition() / Calibrations.climberCals.ticksPerInch;
            if(override){
                if(!enabled){
                    motor.set(ControlMode.PercentOutput, 0);
                }else{
                    motor.set(ControlMode.PercentOutput, overridePower);
                }
            }else{
                if(!enabled){
                    motor.set(ControlMode.PercentOutput, 0);
                    released = false;
                    statusColor = 1;
                }else if(enabled && !released){
                    this.release();
                }else{
                    if(inAuto){
                        target = autoTarget;
                    }else{
                        if(Math.abs(delta) < 0.25 && !tookSnapShot){
                            SnapShot = position;
                            target = SnapShot;
                            tookSnapShot = true;
                        }else if(Math.abs(delta) < 0.25 && tookSnapShot){
                            target = SnapShot;
                        }else{
                            target = position + delta;
                            tookSnapShot = false;
                        }
                        if(Math.abs(target - position) < 1){
                            statusColor = 3;
                        }else{
                            statusColor = 2;
                        }
                    }

                }
                if(target >= Calibrations.climberCals.upperLimit) target = Calibrations.climberCals.upperLimit; 
                motor.set(ControlMode.MotionMagic, (int)(target * Calibrations.climberCals.ticksPerInch));
            }
        }

        public double getCurrentHeight(){
            return position;
        }
    }
    
    public Climber(RobotState state){
        this.robotState = robotState;
        leftClimber = new climberController(Calibrations.CAN_ID.leftClimber);
        rightClimber = new climberController(Calibrations.CAN_ID.rightClimber);
        rachet = new Solenoid(Calibrations.climberCals.rachetID);
    }
    
    

    public void performAction(RobotCommandProvider commander, RobotState state){  
        leftClimber.setOverride(commander.getManualMode());
        rightClimber.setOverride(commander.getManualMode());
        leftClimber.setPower(commander.getLeftClimberDelta() / Calibrations.climberCals.maxDelta);
        rightClimber.setPower(commander.getRightClimberDelta() / Calibrations.climberCals.maxDelta);


        leftClimber.setEnabled(commander.isLeftClimberActivate());
        leftClimber.setDelta(commander.getLeftClimberDelta());
        rightClimber.setEnabled(commander.isRightClimberActivate());
        rightClimber.setDelta(commander.getRightClimberDelta());
        if(commander.isLeftClimberActivate() || commander.isRightClimberActivate()){
            rachet.set(false);
        }else{
            rachet.set(true);
        }

    }

    @Override
    public void updateState() {
        leftClimber.periodic();
        rightClimber.periodic();
        if(leftClimber.getCurrentHeight() > rightClimber.getCurrentHeight()){
            robotState.setHighestClimberPos(leftClimber.getCurrentHeight());
        }else{
            robotState.setHighestClimberPos(rightClimber.getCurrentHeight());
        }
        
    }
    @Override
    public void zeroSensor() {
        leftClimber.reset();
        rightClimber.reset();
    }
    @Override
    public void setSensorValue(Integer value) {
        // TODO Auto-generated method stub
    }
    @Override
    public void setRobotState(RobotState robotState) {
        this.robotState = robotState;
    }
  
}  