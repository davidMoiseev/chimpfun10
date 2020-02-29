package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


public class conveyor {
    private boolean wasStage;
    private boolean frontPorch = false;
    private boolean frontPorchSkip = false;
    private boolean carouselFull = false;
    private boolean shouldStage;
    private double carouselPower = 0.65;
    private double conveyorPower = 0.75;
    private double coveyorPowerStandered = 0.7;
    private double carouselPowerSlow = 0.6;
    private double carouselOutPut = 0;
    private double conveyorOutPut = 0;
    public int reverseTime_1 = 0;
    private int reverseTime_2 = 0;
    private int ballStored;
    private int bCarPos;
    private boolean warning = false;
    private boolean critical = false;
    private boolean IntakelastState;
    private boolean FallingedgeSensor;
    private int Edgecounter = 0;
    public boolean conveyorBounceBack;
    private boolean IntakeCountLockout;
    private boolean Pos4lastState;
    private boolean Pos1lastState;
    private int inConveyor;
    private int inCarousel;
    private int carouselPos = 2;
    public PowerDistributionPanel powerPannel;
    DigitalInput pos1Sensor;
    DigitalInput pos2Sensor;
    DigitalInput pos3Sensor;
    DigitalInput pos4Sensor;
    DigitalInput pos5Sensor;
    DigitalInput IntakeSensor;
    VictorSPX conveyorMotor;
    VictorSPX carouselMotor;
    private boolean IntakeOn;
    private int StatusLightState;

    public conveyor() {
        IntakeSensor = new DigitalInput(0);
        pos1Sensor = new DigitalInput(5);
        pos2Sensor = new DigitalInput(4);
        pos3Sensor = new DigitalInput(3);
        pos4Sensor = new DigitalInput(2);
        pos5Sensor = new DigitalInput(1);
        carouselMotor = new VictorSPX(Calibrations.CAN_ID.carousel);
        conveyorMotor = new VictorSPX(Calibrations.CAN_ID.conveyor);
        powerPannel = new PowerDistributionPanel();
        carouselMotor.setNeutralMode(NeutralMode.Brake);
        conveyorMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void setIntakeOn(boolean intakeOn) {
        this.IntakeOn = intakeOn;
    }

    public void setManualInputs(double conveyorPwr, double carouselPwr){
        conveyorOutPut = conveyorPwr;
        carouselOutPut = carouselPwr;
    }

    public void conveyorAutoFeed() {
        //This is the conveyor run logic that determins when it is the correct time to run the conveyor
        if (inConveyor == 1 && carouselFull){
            if (pos5Sensor.get() == true){
                conveyorOutPut = 0;
                conveyorBounceBack = false;
                IntakeCountLockout = false;
            }else{
                if(pos4Sensor.get() || conveyorBounceBack){
                    conveyorOutPut = -conveyorPower;
                    IntakeCountLockout = true;
                    conveyorBounceBack = true;
                }else{
                    IntakeCountLockout = false;
                    conveyorOutPut = conveyorPower; 
                }
            }
        }else if(inConveyor >= 2 && carouselFull){
            if (pos4Sensor.get() == true){
                conveyorOutPut = 0;
            }else{
                conveyorOutPut = conveyorPower; 
            }
        }else if(inConveyor > 0 || IntakeOn || shouldStage){
            IntakeCountLockout = false;
            conveyorOutPut = conveyorPower; 
        }else{
            IntakeCountLockout = false;
            conveyorOutPut = 0; 
        }
    }
    public void carouselState(){
        if(((carouselPos <= 0 && pos1Sensor.get()) || (inConveyor >= 3)) && (shouldStage == false)){
            carouselFull = true;
        }else{
            carouselFull = false;
        }
    }

    public boolean carouselStage(){
        boolean staging = true;
        switch(carouselPos){
            case 2:
                if(pos3Sensor.get() == false){
                    carouselOutPut = carouselPower;
                }else{
                    carouselPos--;
                    inConveyor--;
                    staging = false;
                }
            break;
            case 1:
                if(pos2Sensor.get() == false){
                    carouselOutPut = carouselPower;
                }else{
                    carouselPos--;
                    inConveyor--;
                    staging = false;
                }
            break;
            case 0:
                if(Edgecounter < 2 && (!pos1Sensor.get() || frontPorchSkip)){
                    frontPorchSkip = true;
                    if(FallingedgeSensor != pos2Sensor.get()){
                        Edgecounter++;
                    }
                    FallingedgeSensor = pos2Sensor.get();
                    carouselOutPut = carouselPowerSlow;
                }else{
                    inConveyor--;
                    staging = false;
                    frontPorchSkip = false;
                }
            break;
            case -1:
                if(pos3Sensor.get() == false){
                    carouselOutPut = -carouselPower;
                }else{
                    carouselPos = 1;
                    staging = false;
                }
            break;
        }
        if(staging == false){
            carouselOutPut = 0;
            Edgecounter = 0;
        }
        return staging;
    }

    public void carouselPrime(){
        if(pos1Sensor.get() == false){
            carouselOutPut = carouselPower;
        }else{
            if(carouselPos == 1){
                carouselPos = -1;
            }
            if(carouselPos == 2){
                carouselPos = -2;
            }
            carouselOutPut = 0;
        }
    }

    public void carouselStageCheck(){
        if ((pos4Sensor.get() && inConveyor > 0 && (carouselFull == false)) || carouselPos < 0){
            shouldStage = true;
        }
        if (shouldStage){
            shouldStage = carouselStage();
            if(!wasStage){
                inCarousel++;
            }
            wasStage = true;
        }else{
            FallingedgeSensor = pos2Sensor.get();
            carouselOutPut = 0;
            conveyorPower = coveyorPowerStandered;
            wasStage = false;
        }
        
    }

    public void shootPower(double pwr){
        conveyorOutPut = -(pwr - 0.2);   
        carouselOutPut = -pwr;
        if(pos4Sensor.get() != Pos4lastState){
            if(pos4Sensor.get() != Pos4lastState){
                inConveyor--;
            }else{
                Pos4lastState = pos4Sensor.get();
            }
        }
    }

    public void shotFired(){
        if(pos1Sensor.get() != Pos1lastState){
            if(pos1Sensor.get() != true){
                ballStored--;
                inCarousel--;
            }
        }
        if(ballStored <= 0){
            this.reset();
        }
        Pos1lastState = pos1Sensor.get();
    }

    public enum feedModes{
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
    public void stage(feedModes mode){
        warning = false;
        switch(mode){
            case autoFill:
                this.count(true);
                this.carouselState();
                this.conveyorAutoFeed();
                this.carouselStageCheck();
                this.antiJam();
            break;
            case prime:
                this.count(true);
                this.conveyorAutoFeed();
                this.carouselPrime();
            break;
            case shoot:
                this.count(true);
                this.shootPower(-0.45); //to const
                this.shotFired();
            break;
            case reject:
                this.antiJam();
                this.count(false);
                this.shootPower(0.5); //to const
            break;
            case stop:
                this.count(true);
                this.shootPower(0);
            break;
            case confirm:
                carouselPos = 0;
                ballStored = 3;
                inConveyor = 0;
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

    public void reset(){
        ballStored = 0;
        inConveyor = 0;
        carouselOutPut = 0;
        carouselPos = 2;
        shouldStage = false;
        IntakeCountLockout = false;
        conveyorBounceBack = false;
        inCarousel = 0;
    }

    public boolean inWarning(){
        return warning;
    }

    public boolean inCritical(){
        return critical;
    }

    public void testState(){
    
    }
    
    public int getStatusLightState(){
        return StatusLightState;
    }

    public int BallStored(){
        return ballStored;
    }

    public void antiJam(){
        if(powerPannel.getCurrent(7) > 20 ){
            reverseTime_1 = 75;
        }else{
            reverseTime_1--;
        }
        if(powerPannel.getCurrent(6) > 30){
            reverseTime_2 = 75;
        }else{
            reverseTime_2--;
        }

        if(reverseTime_1 > 0){
            conveyorOutPut = 0;
            warning = true;
        }
        if(reverseTime_2 > 0){
            carouselOutPut = 0;
            warning = true;
            conveyorOutPut = 0;
        }
    }

    public void count(boolean direction){
        if (direction){
            if(IntakeSensor.get() != IntakelastState && IntakeCountLockout == false){
                if(IntakeSensor.get() == true){
                    ballStored++;
                    inConveyor++;
                }
                IntakelastState = IntakeSensor.get();
            }
        }else{
            if(IntakeSensor.get() != IntakelastState && IntakeCountLockout == false){
                if(IntakeSensor.get() == true){
                    ballStored--;
                    inConveyor--;
                }
                IntakelastState = IntakeSensor.get();
            }
        }        
        if(ballStored > 5){
            warning = true;
        }

    }


    public void runMotors(){
        carouselMotor.set(ControlMode.PercentOutput, -carouselOutPut);
        conveyorMotor.set(ControlMode.PercentOutput, conveyorOutPut);
    }


    public void display(){
        if(warning || critical){
            StatusLightState = 0;
        }else if(pos1Sensor.get() == true && ballStored > 1){
            StatusLightState = 3;
        }else if(conveyorOutPut != 0 || carouselOutPut != 0){
            StatusLightState = 2;
        }else{
            StatusLightState = 1;
        }
        SmartDashboard.putBoolean("Sensor#1", IntakeSensor.get());
        SmartDashboard.putBoolean("Sensor#2", pos5Sensor.get());
        SmartDashboard.putBoolean("Sensor#3", pos4Sensor.get());
        SmartDashboard.putBoolean("Sensor#4", pos3Sensor.get());
        SmartDashboard.putBoolean("Sensor#5", pos2Sensor.get());
        SmartDashboard.putBoolean("Sensor#6", pos1Sensor.get());
        SmartDashboard.putBoolean("Staging", shouldStage);
        SmartDashboard.putNumber("Inventory Total",ballStored);
        SmartDashboard.putNumber("Conveyor Total",inConveyor);
        SmartDashboard.putNumber("Carousel Total",inCarousel);
        SmartDashboard.putBoolean("Carousel Full", carouselFull);
        SmartDashboard.putNumber("Carousel Pos",carouselPos);
        SmartDashboard.putNumber("Conveyor OutPut", conveyorOutPut);
        SmartDashboard.putNumber("Bype Carousel Possible",bCarPos);
        
        SmartDashboard.putNumber("Carousel Output", carouselMotor.getMotorOutputPercent());
    }

    public void updateStatusLight(){
        SmartDashboard.putNumber("ConveyerOutputStatus",StatusLightState);
    }
    
    public void confirmInventory(){
        ballStored = 0;
        if(IntakeSensor.get()) ballStored++;
        if(pos1Sensor.get()) ballStored++;
        if(pos2Sensor.get()) ballStored++;
        if(pos3Sensor.get()) ballStored++;
        if(pos4Sensor.get()) ballStored++;
        if(pos5Sensor.get()) ballStored++;
        bCarPos = 0; //ByteBased Carousel Position
        // if(pos1Sensor.get() == false) bCarPos = bCarPos + 1;
        if(pos2Sensor.get() == false) bCarPos = bCarPos + 3;
        if(pos3Sensor.get()) bCarPos = bCarPos + 5;
    }

}        
