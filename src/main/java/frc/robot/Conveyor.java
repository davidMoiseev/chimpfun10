package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.hotutilites.hotlogger.HotLogger;

public class Conveyor {
    private boolean wasStage;
    private final boolean frontPorch = false;
    private boolean frontPorchSkip = false;
    private boolean carouselFull = false;
    private boolean shouldStage;
    private boolean full;
    private final double carouselPower = 0.8;
    private final double conveyorPower = 0.75;  //previously 0.6
    private double carouselOutPut = 0;
    private double conveyorOutPut = 0;
    public int reverseTime_1 = 0;
    private int reverseTime_2 = 0;
    private int ballStored;
    private int bCarPos;
    private boolean warning = false;
    private final boolean critical = false;
    private boolean IntakeLastState;
    private boolean FallingEdgeSensor;
    private int EdgeCounter = 0;
    public boolean conveyorBounceBack;
    private int conveyorBounceBackTimeOut = 0;
    private boolean IntakeCountLockout;
    private boolean Pos4lastState;
    private boolean Pos1lastState;
    private int inConveyor;
    private int inCarousel;
    private int carouselPos = 2;
    public PowerDistributionPanel powerPanel;
    DigitalInput pos1Sensor;
    DigitalInput pos2Sensor;
    DigitalInput pos3Sensor;
    DigitalInput pos4Sensor;
    DigitalInput pos5Sensor;
    DigitalInput IntakeSensor;
    CANSparkMax conveyorMotor;
    CANSparkMax carouselMotor;
    CANEncoder conveyorEncoder;
    CANEncoder carouselEncoder;
    MedianFilter conveyorSmooth;
    MedianFilter carouselSmooth;
    private final double CarouselAverageCurrent;
    private boolean IntakeOn;
    private int StatusLightState;

    public Conveyor() {
        IntakeSensor = new DigitalInput(0);
        pos1Sensor = new DigitalInput(5);
        pos2Sensor = new DigitalInput(4);
        pos3Sensor = new DigitalInput(3);
        pos4Sensor = new DigitalInput(2);
        pos5Sensor = new DigitalInput(1);
        carouselMotor = new CANSparkMax(Calibrations.CAN_ID.carousel, MotorType.kBrushless);
        conveyorMotor = new CANSparkMax(Calibrations.CAN_ID.conveyor, MotorType.kBrushless);
        CarouselAverageCurrent = 0;
        conveyorEncoder = new CANEncoder(conveyorMotor);
        carouselEncoder = new CANEncoder(carouselMotor);
        conveyorMotor.setSmartCurrentLimit(20);
        carouselMotor.setSmartCurrentLimit(20);
        conveyorSmooth = new MedianFilter(10);
        carouselSmooth = new MedianFilter(10);
        powerPanel = new PowerDistributionPanel();
        carouselMotor.setIdleMode(IdleMode.kBrake);
        conveyorMotor.setIdleMode(IdleMode.kBrake);
        conveyorMotor.setInverted(true);
    }

    public boolean isFull() {
        if (carouselFull && inConveyor >= 2) {
            full = true;
            return true;
        } else {
            full = false;
            return false;
        }

    }

    public void setIntakeOn(boolean intakeOn) {
        this.IntakeOn = intakeOn;
    }

    public void setManualInputs(double conveyorPwr, double carouselPwr) {
        conveyorOutPut = conveyorPwr;
        carouselOutPut = carouselPwr;
    }

    public void conveyorAutoFeed() {
        //This is the conveyor run logic that determins when it is the correct time to run the conveyor
        if (inConveyor == 1 && carouselFull) {
            if (pos5Sensor.get()) {
                conveyorOutPut = 0;
                conveyorBounceBack = false;
                IntakeCountLockout = false;
                conveyorBounceBackTimeOut = 0;
            } else {
                if ((pos4Sensor.get() || conveyorBounceBack) && conveyorBounceBackTimeOut <= 75) {
                    conveyorOutPut = -conveyorPower;
                    IntakeCountLockout = true;
                    conveyorBounceBack = true;
                    conveyorBounceBackTimeOut++;
                } else {
                    IntakeCountLockout = false;
                    conveyorOutPut = conveyorPower;
                    // conveyorBounceBackTimeOut = 0; 
                }
            }
        } else if (inConveyor >= 2 && carouselFull) {
            if (pos4Sensor.get()) {
                conveyorOutPut = 0;
            } else {
                conveyorOutPut = conveyorPower;
            }
        } else if (inConveyor > 0 || IntakeOn || shouldStage) {
            IntakeCountLockout = false;
            conveyorOutPut = conveyorPower;
        } else {
            IntakeCountLockout = false;
            conveyorOutPut = 0;
        }
    }

    public void carouselState() {
        carouselFull = ((carouselPos <= 0 && pos1Sensor.get()) || (inConveyor >= 3)) && (!shouldStage);
    }

    public boolean carouselStage() {
        boolean staging = true;
        switch (carouselPos) {
            case 2:
                if (!pos3Sensor.get()) {
                    carouselOutPut = carouselPower;
                } else {
                    carouselPos--;
                    inConveyor--;
                    staging = false;
                }
                break;
            case 1:
                if (!pos2Sensor.get()) {
                    carouselOutPut = carouselPower;
                } else {
                    carouselPos--;
                    inConveyor--;
                    staging = false;
                }
                break;
            case 0:
                if (EdgeCounter < 2 && (!pos1Sensor.get() || frontPorchSkip)) {
                    frontPorchSkip = true;
                    if (FallingEdgeSensor != pos2Sensor.get()) {
                        EdgeCounter++;
                    }
                    FallingEdgeSensor = pos2Sensor.get();
                    carouselOutPut = carouselPower;
                } else {
                    inConveyor--;
                    staging = false;
                    frontPorchSkip = false;
                }
                break;
            case -1:
                if (!pos3Sensor.get()) {
                    carouselOutPut = -carouselPower;
                } else {
                    carouselPos = 1;
                    staging = false;
                }
                break;
        }
        if (!staging) {
            carouselOutPut = 0;
            EdgeCounter = 0;
        }
        return staging;
    }

    public void carouselPrime() {
        if (!pos1Sensor.get()) {
            carouselOutPut = carouselPower;
            conveyorOutPut = conveyorPower;
        } else {
            conveyorOutPut = 0;
            carouselOutPut = 0;
        }
    }

    public void carouselStageCheck() {
        if ((pos4Sensor.get() && inConveyor > 0 && (!carouselFull))) {
            shouldStage = true;
        }
        if (shouldStage) {
            shouldStage = carouselStage();
            if (!wasStage) {
                inCarousel++;
            }
            wasStage = true;
        } else {
            FallingEdgeSensor = pos2Sensor.get();
            wasStage = false;
        }

    }

    public void zeroMotors() {
        carouselOutPut = 0;
        conveyorOutPut = 0;
    }

    public void shootPower(double pwr) {
        conveyorOutPut = conveyorPower * pwr;
        carouselOutPut = carouselPower * pwr;
        if (pos4Sensor.get() != Pos4lastState) {
            if (pos4Sensor.get() != Pos4lastState) {
                inConveyor--;
            } else {
                Pos4lastState = pos4Sensor.get();
            }
        }
    }

    public void shotFired() {
        if (pos1Sensor.get() != Pos1lastState) {
            if (!pos1Sensor.get()) {
                ballStored--;
                inCarousel--;
            }
        }
        if (ballStored <= 0) {
            this.reset();
        }
        Pos1lastState = pos1Sensor.get();
    }

    public enum feedModes {
        autoFill,
        prime,
        shoot,
        reject,
        confirm,
        stop,
        reset,
        manual,
        test,
        manualChangeReset
    }

    public void stage(feedModes mode) {
        warning = false;
        switch (mode) {
            case autoFill:
                this.zeroMotors();
                this.count(true);
                this.carouselState();
                this.conveyorAutoFeed();
                this.carouselStageCheck();
                this.antiJam();
                break;
            case prime:
                this.zeroMotors();
                this.count(true);
                this.conveyorAutoFeed();
                this.carouselPrime();
                break;
            case shoot:
                this.antiJam();
                this.zeroMotors();
                this.shotFired();
                this.count(true);
                this.shootPower(0.85); //to const
                break;
            case reject:
                this.zeroMotors();
                this.antiJam();
                this.count(false);
                this.shootPower(-1); //to const
                break;
            case stop:
                this.zeroMotors();
                this.count(true);
                break;
            case confirm:
                this.confirmInventory();
                break;
            case reset:
                this.reset();
                break;
            case manual:
                break;
            case test:
                break;
            case manualChangeReset:
                conveyorOutPut = 0;
                carouselOutPut = 0;
                shouldStage = false;
                IntakeCountLockout = false;
                conveyorBounceBack = false;
                break;
        }
        runMotors();
        if (carouselPos < -2) carouselPos = 2;
        //if (carouselPos < 2) carouselPos = -2;
    }

    public void reset() {
        ballStored = 0;
        inConveyor = 0;
        carouselOutPut = 0;
        carouselPos = 2;
        shouldStage = false;
        IntakeCountLockout = false;
        conveyorBounceBack = false;
        inCarousel = 0;
        conveyorBounceBackTimeOut = 0;
    }

    public boolean inWarning() {
        return warning;
    }

    public boolean inCritical() {
        return critical;
    }

    public void testState() {

    }

    public int getStatusLightState() {
        return StatusLightState;
    }

    public int BallStored() {
        return ballStored;
    }

    public void antiJam() {
        double conveyorMedian = conveyorSmooth.calculate(conveyorMotor.getOutputCurrent());
        double carouselMedian = carouselSmooth.calculate(carouselMotor.getOutputCurrent());
        if (conveyorMedian >= 25 && conveyorEncoder.getVelocity() < 150) {
            reverseTime_1 = 75;
        } else {
            reverseTime_1--;
        }
        if (carouselMedian >= 25 && carouselEncoder.getVelocity() < 250) {
            reverseTime_2 = 75;
        } else {
            reverseTime_2--;
        }

        if (reverseTime_1 > 0) {
            conveyorOutPut = 0;
            warning = true;
        }
        if (reverseTime_2 > 0) {
            carouselOutPut = 0;
            warning = true;
            conveyorOutPut = 0;
        }
    }

    public void count(boolean direction) {
        if (direction) {
            if (IntakeSensor.get() != IntakeLastState && !IntakeCountLockout) {
                if (IntakeSensor.get()) {
                    ballStored++;
                    inConveyor++;
                }
                IntakeLastState = IntakeSensor.get();
            }
        } else {
            if (IntakeSensor.get() != IntakeLastState && !IntakeCountLockout) {
                if (IntakeSensor.get()) {
                    ballStored--;
                    inConveyor--;
                }
                IntakeLastState = IntakeSensor.get();
            }
        }
        if (ballStored > 5) {
            warning = true;
        }

    }


    public void runMotors() {
        carouselMotor.set(-carouselOutPut);
        conveyorMotor.set(conveyorOutPut);
    }


    public void display() {
        if (warning || critical) {
            StatusLightState = 0;
        } else if (pos1Sensor.get() && ballStored > 1) {
            StatusLightState = 3;
        } else if (conveyorOutPut != 0 || carouselOutPut != 0) {
            StatusLightState = 2;
        } else {
            StatusLightState = 1;
        }
        SmartDashboard.putBoolean("Sensor#1", IntakeSensor.get());
        SmartDashboard.putBoolean("Sensor#2", pos5Sensor.get());
        SmartDashboard.putBoolean("Sensor#3", pos4Sensor.get());
        SmartDashboard.putBoolean("Sensor#4", pos3Sensor.get());
        SmartDashboard.putBoolean("Sensor#5", pos2Sensor.get());
        SmartDashboard.putBoolean("Sensor#6", pos1Sensor.get());
        SmartDashboard.putBoolean("Staging", shouldStage);
        SmartDashboard.putNumber("Inventory Total", ballStored);
        SmartDashboard.putNumber("Conveyor Total", inConveyor);
        SmartDashboard.putNumber("Carousel Total", inCarousel);
        SmartDashboard.putBoolean("Carousel Full", carouselFull);
        SmartDashboard.putNumber("Carousel Pos", carouselPos);
        SmartDashboard.putNumber("Conveyor OutPut", conveyorOutPut);
        SmartDashboard.putNumber("Carousel Output", carouselOutPut);

        HotLogger.Log("conveyor output", conveyorOutPut);
        HotLogger.Log("carousel output", carouselOutPut);
    }

    public void updateStatusLight() {
        SmartDashboard.putNumber("ConveyerOutputStatus", StatusLightState);
    }

    public void confirmInventory() {
        ballStored = 3;
        inConveyor = 0;
        inCarousel = 3;
        carouselPos = 0;

    }

}        
