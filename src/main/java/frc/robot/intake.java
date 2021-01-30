package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class intake {

    private final TalonSRX lifterMotor;
    private final TalonSRX intake_1;
    private final CANSparkMax intake_2;
    private final int reverseTime_1 = 0;
    private final int reverseTime_2 = 0;
    private final int groundPosition = 50;

    public intake() {
        lifterMotor = new TalonSRX(Calibrations.CAN_ID.intakeLifter);
        intake_1 = new TalonSRX(Calibrations.CAN_ID.intakeMotor1);
        intake_1.setInverted(true);
        intake_2 = new CANSparkMax(Calibrations.CAN_ID.intakeMotor2, MotorType.kBrushless);
        intake_1.configPeakCurrentLimit(50);
        intake_1.configPeakCurrentDuration(500);
        intake_1.configContinuousCurrentLimit(30);
        intake_2.setSmartCurrentLimit(20);
        lifterMotor.configPeakCurrentLimit(45, 10); //Limit of 45 amps
        lifterMotor.configPeakCurrentDuration(500, 10); //For 500ms
        lifterMotor.configContinuousCurrentLimit(30, 10); //Limit of 30 amps cont

    }

    public boolean readyForPickup() {
        return lifterMotor.getSelectedSensorPosition() < groundPosition;
    }

    public void setposition(IntakePositions position) {
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

    public void consume(double power) {
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

        if (reverseTime_1 > 0) {
            intake_1.set(ControlMode.PercentOutput, -power);
        } else {
            intake_1.set(ControlMode.PercentOutput, power);
        }
        if (reverseTime_2 > 0) {
            intake_2.set(power);
        } else {
            intake_2.set(-power);
        }

    }

    public void read() {

    }

    public enum IntakePositions {
        packaged,
        limelite,
        ground,
        autoshot
    }
}       