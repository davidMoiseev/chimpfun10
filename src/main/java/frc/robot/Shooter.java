package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;

public class Shooter {
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
    public double lP, lI, lD, lIz, lFF, lMaxOutput, lMinOutput, lmaxRPM;
    private final boolean inFault = false;
    public Compressor compressor;
    public Solenoid hood1;
    public Solenoid hood2;
    public CANSparkMax m_motor;
    public CANSparkMax m_mator;
    public CANSparkMax m_feeder;
    public CANEncoder m_encoder;
    public CANEncoder m_encoder2;
    public CANPIDController m_pidController;
    public double PIDTarget;
    public boolean Ready;
    public int Stabalizer;
    public double IAccum;
    public double rpm;
    public double rpm2;
    public PowerDistributionPanel powerPannel;
    public double lowVoltage;
    public double lowVoltageCnt;
    public double lowVoltageLockout;

    public Shooter() {
        compressor = new Compressor(Calibrations.CAN_ID.pcm);
        compressor.setClosedLoopControl(true);
        hood1 = new Solenoid(6);

        if (Calibrations.isCompBot) {
            hood2 = new Solenoid(5);
        } else {
            hood2 = new Solenoid(7);
        }
        powerPannel = new PowerDistributionPanel();
        kP = Calibrations.shooter_PID.kP;
        kI = Calibrations.shooter_PID.kI;
        kD = Calibrations.shooter_PID.kD;
        kIz = Calibrations.shooter_PID.kIz;
        kFF = Calibrations.shooter_PID.kFF;
        kMaxOutput = Calibrations.shooter_PID.kMaxOutput;
        kMinOutput = Calibrations.shooter_PID.kMinOutput;

        lP = Calibrations.shooter2_PID.kP;
        lI = Calibrations.shooter2_PID.kI;
        lD = Calibrations.shooter2_PID.kD;
        lIz = Calibrations.shooter2_PID.kIz;
        lFF = Calibrations.shooter2_PID.kFF;

        m_motor = new CANSparkMax(Calibrations.CAN_ID.shooter1, MotorType.kBrushless);
        m_mator = new CANSparkMax(Calibrations.CAN_ID.shooter2, MotorType.kBrushless);
        m_feeder = new CANSparkMax(Calibrations.CAN_ID.indexer, MotorType.kBrushless);
        m_feeder.setSmartCurrentLimit(20);
        m_feeder.setInverted(!Calibrations.hardware.indexerInvert);
        m_encoder = m_motor.getEncoder();
        m_encoder2 = m_mator.getEncoder();
        m_pidController = m_motor.getPIDController();
        m_pidController.setFeedbackDevice(m_encoder);
        m_pidController.setP(kP, 0);
        m_pidController.setI(kI, 0);
        m_pidController.setD(kD, 0);
        m_pidController.setIZone(kIz, 0);
        m_pidController.setFF(kFF, 0);

        m_pidController.setP(lP, 1);
        m_pidController.setI(lI, 1);
        m_pidController.setD(lD, 1);
        m_pidController.setIZone(lIz, 1);
        m_pidController.setFF(lFF, 1);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
        m_motor.setInverted(false);
        m_mator.follow(m_motor, true);
    }

    public void manual(double pwr) { // Allow manual control of the motor
        m_pidController.setReference(pwr, ControlType.kDutyCycle);
        PIDTarget = 0;
    }

    public boolean isInFault() { // Make the cool dashboard light go red when voltage power modifier drops too low
        return lowVoltage < 0.95;
    }

    public void setCompressor(boolean enabled) {
        compressor.setClosedLoopControl(enabled);
    }

    public void stopPIDMotor() { // Stop motor integrated pid control instead of setting RPM to zero which would still
        m_pidController.setReference(0, ControlType.kDutyCycle); // leave motor powered
        PIDTarget = 0;
        m_pidController.setOutputRange(0, kMaxOutput);
        setCompressor(true);
    }

    public void PIDmotor(double rpm) { // Motor PID control
        setCompressor(false);
        m_pidController.setOutputRange(kMinOutput, lowVoltage);
        if (rpm > 3400) {
            rpm = rpm * 24.0 / 36.0;
            PIDTarget = rpm;
            m_pidController.setReference(rpm, ControlType.kVelocity, 0);
        } else {
            rpm = rpm * 24.0 / 36.0;
            PIDTarget = rpm;
            m_pidController.setReference(rpm, ControlType.kVelocity, 1);

        }

    }

    public void PowerCheck() { // Reduce shooter power if voltage begins to drop below min value to prevent hard brownout
        if (powerPannel.getVoltage() < 9.5) {
            lowVoltageCnt = (powerPannel.getVoltage() - 7.4) / 2.2;
            if (lowVoltageCnt < lowVoltage) {
                lowVoltage = lowVoltageCnt;
            } else {
                lowVoltage = lowVoltage + 0.05;
            }
        } else {
            lowVoltage = lowVoltage + 0.05;
        }
        if (lowVoltage >= 1) lowVoltage = 1;
    }

    public boolean isShooterStable() { // Never finished
        return Ready;
    }

    public void indexPower(double pwr) {
        m_feeder.set(pwr);
    }

    public void read() { // Update telemetry values every control cycle
        this.PowerCheck();
        rpm = Math.abs(m_encoder.getVelocity() * (36.0 / 24.0));
        rpm2 = Math.abs(m_encoder2.getVelocity() * (36.0 / 24.0));
        IAccum = m_pidController.getIAccum();
        if ((Math.abs(PIDTarget * (36.0 / 24.0) - rpm)) < 100) {
            Stabalizer++;
        } else {
            Stabalizer = 0;
        }
        Ready = Stabalizer > 10;
    }

    public enum HoodPosition {
        goingUnder,
        trench,
        wallShot,
        oneBotBack,
        autoshot
    }

    public void setHood(HoodPosition pos) {
        switch (pos) {
            case wallShot:
            case goingUnder: //trench
                hood1.set(Calibrations.hardware.shortPistonExtend); //short extened
                hood2.set(Calibrations.hardware.longPistonExtend); //long extented
                break;
            case trench:
                hood1.set(!Calibrations.hardware.shortPistonExtend); //short 
                hood2.set(!Calibrations.hardware.longPistonExtend); //long
                break;
            case oneBotBack:
                hood1.set(Calibrations.hardware.shortPistonExtend); //short 
                hood2.set(!Calibrations.hardware.longPistonExtend); //long
                break;
            case autoshot:
                if (Calibrations.isCompBot) {
                    hood1.set(!Calibrations.hardware.shortPistonExtend); //short
                    hood2.set(Calibrations.hardware.longPistonExtend); //long
                } else {
                    hood1.set(Calibrations.hardware.shortPistonExtend); //short
                    hood2.set(!Calibrations.hardware.longPistonExtend); //long
                }
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + pos);
        }
    }

    public void updateStatus() { // Update the status light, and hotlog
        HotLogger.Log("Target Speed", PIDTarget);
        HotLogger.Log("Shooter PowerOutput", m_motor.getAppliedOutput());
        HotLogger.Log("shooter speed", rpm);
        HotLogger.Log("Battery Voltage", powerPannel.getVoltage());
        HotLogger.Log("Current Current Draw", powerPannel.getTotalCurrent());
        int status;
        if (lowVoltage < 0.9) {
            status = 0;
        } else if (Ready && PIDTarget != 0) {
            status = 3;
        } else if (PIDTarget != 0) {
            status = 2;
        } else {
            status = 1;
        }
        SmartDashboard.putNumber("ShooterOutputStatus", status);
    }

    public void display() { // Update data to the Hot dashboard
        SmartDashboard.putNumber("I accumlator", IAccum);
        SmartDashboard.putNumber("Amp Draw", m_motor.getOutputCurrent());
        SmartDashboard.putNumber("Voltage Modfyer", lowVoltage);
        SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
        SmartDashboard.putNumber("Encoder Velocity", rpm);
        SmartDashboard.putNumber("Encoder Velocity2", rpm2);
        SmartDashboard.putNumber("Combined Velocity", (rpm + rpm2) / 2);
        SmartDashboard.putNumber("Encoder Velocity Difference", rpm - rpm2);
        SmartDashboard.putNumber("PID Error", (PIDTarget * (36.0 / 24.0)) - rpm);
        SmartDashboard.putNumber("OutPut Power", m_motor.getAppliedOutput());
        SmartDashboard.putBoolean("Ready?", Ready);
    }

}    
