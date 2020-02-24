package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class intake {

    private TalonSRX lifterMotor;
    private TalonSRX intake_1;
    private TalonSRX intake_2;
    private int reverseTime_1 = 0;
    private int reverseTime_2 = 0;
    private int groundPosition = 50;
    public intake(){
        lifterMotor = new TalonSRX(Calibrations.CAN_ID.intakeLifter);
        intake_1 = new TalonSRX(Calibrations.CAN_ID.intakeMotor1);
        intake_1.setInverted(true);
        intake_2 = new TalonSRX(Calibrations.CAN_ID.intakeMotor2);
        intake_1.configPeakCurrentLimit(50);
        intake_1.configPeakCurrentDuration(500);
        intake_1.configContinuousCurrentLimit(30);
        intake_2.configPeakCurrentLimit(50);
        intake_2.configPeakCurrentDuration(500);
        intake_2.configContinuousCurrentLimit(30);
        lifterMotor.configPeakCurrentLimit(45, 10); //Limit of 45 amps
        lifterMotor.configPeakCurrentDuration(500, 10); //For 500ms
        lifterMotor.configContinuousCurrentLimit(30, 10); //Limit of 30 amps cont
        
    }

    public enum IntakePositions{
        packaged,
        limelite,
        ground,
        autoshot 
    }

    public boolean readyForPickup(){
        if(lifterMotor.getSelectedSensorPosition() < groundPosition){
            return true;
        }else{
            return false;
        }
    }

    public void setposition(IntakePositions position){
        // switch(position){
        //     case packaged:
        //        liftMotorPID.setReference(250, ControlType.kSmartMotion); 
        //     break;
        //     case limelite:
        //        liftMotorPID.setReference(100, ControlType.kSmartMotion);
        //     break;
        //     case ground:
        //         liftMotorPID.setReference(0, ControlType.kSmartMotion);
        //     break;
        //     case autoshot:
        //         liftMotorPID.setReference(175, ControlType.kSmartMotion);
        //     break;
        // }
    }

    public void consume(double power){
        // if(intake_1.getStatorCurrent() > 30){
        //     reverseTime_1 = 25; 
        // }else{
        //     reverseTime_1--;
        // }
        // if(intake_2.getStatorCurrent() > 30){
        //     reverseTime_2 = 25;
        // }else{
        //     reverseTime_2--;
        // }

        if(reverseTime_1 > 0){
            intake_1.set(ControlMode.PercentOutput, -power); 
        }else{
            intake_1.set(ControlMode.PercentOutput, power); 
        }
        if(reverseTime_2 > 0){
            intake_2.set(ControlMode.PercentOutput, -power); 
        }else{
            intake_2.set(ControlMode.PercentOutput, power); 
        }

    }

    public void read(){

    }
}       