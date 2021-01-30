package frc.robot;

import com.ctre.phoenix.CANifier;

import org.hotutilites.hotInterfaces.IHotSensedActuator;
import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDController implements IHotSensedActuator<RobotState, RobotCommandProvider, Integer> {
    public static CANifier canifier = new CANifier(20);
    public double flashDelayTimer;
    public double flashTime = Calibrations.ballSuperviserVals.ledCycleTime;
    public double dutyCycle = Calibrations.ballSuperviserVals.ledDutyCycle;
    public double dimmer = 1;
    public int StartState = 0;
    public int POSTTimer = 50;
    public int POSTTime = 50;
    public boolean POSTFinished = false;
    private RobotState robotState;
    private final PowerDistributionPanel powerPannel;

    public LEDController(final RobotState robotState) {
        this.robotState = robotState;
        powerPannel = new PowerDistributionPanel();
    }

    private void POSTRoutine() {
        switch (StartState) {
            case 0:
                setLEDcolor(5);
                POSTTime = 85;
                break;
            case 1:
                setLEDcolor(3);
                POSTTime = 40;
                break;
            case 2:
                setLEDcolor(2);
                POSTTime = 40;
                break;
            case 3:
                setLEDcolor(0);
                POSTTime = 40;
                break;
            case 4:
                setLEDcolor(-1);
                POSTTime = 50;
                break;
            case 5:
                POSTFinished = true;
                break;
        }
        if (POSTTimer >= POSTTime) {
            POSTTimer = 0;
            StartState++;
        } else {
            POSTTimer++;
        }
    }


    private void setLEDcolor(int color) {
        SmartDashboard.putNumber("MasterIndicator", color);
        HotLogger.Log("MasterIndicator State", color);
        switch (color) {
            case 5: //White
                canifier.setLEDOutput(1 * dimmer, CANifier.LEDChannel.LEDChannelA);  //Green
                canifier.setLEDOutput(1 * dimmer, CANifier.LEDChannel.LEDChannelB);  //Red
                canifier.setLEDOutput(1 * dimmer, CANifier.LEDChannel.LEDChannelC);  //Blue
                break;
            case 4: //Purple
                canifier.setLEDOutput(0 * dimmer, CANifier.LEDChannel.LEDChannelA);  //Green
                canifier.setLEDOutput(1 * dimmer, CANifier.LEDChannel.LEDChannelB);  //Red
                canifier.setLEDOutput(1 * dimmer, CANifier.LEDChannel.LEDChannelC);  //Blue
                break;
            case 3: //green
                canifier.setLEDOutput(1 * dimmer, CANifier.LEDChannel.LEDChannelA);  //Green
                canifier.setLEDOutput(0 * dimmer, CANifier.LEDChannel.LEDChannelB);  //Red
                canifier.setLEDOutput(0 * dimmer, CANifier.LEDChannel.LEDChannelC);  //Blue
                break;
            case 2: //Blue
                canifier.setLEDOutput(0 * dimmer, CANifier.LEDChannel.LEDChannelA);  //Green
                canifier.setLEDOutput(0 * dimmer, CANifier.LEDChannel.LEDChannelB);  //Red
                canifier.setLEDOutput(1 * dimmer, CANifier.LEDChannel.LEDChannelC);  //Blue
                break;
            case 1: //Yellow
                canifier.setLEDOutput(0.5 * dimmer, CANifier.LEDChannel.LEDChannelA);  //Green
                canifier.setLEDOutput(1 * dimmer, CANifier.LEDChannel.LEDChannelB);  //Red
                canifier.setLEDOutput(0 * dimmer, CANifier.LEDChannel.LEDChannelC);  //Blue
                break;
            case 0: //Red
                canifier.setLEDOutput(0 * dimmer, CANifier.LEDChannel.LEDChannelA);  //Green
                canifier.setLEDOutput(1 * dimmer, CANifier.LEDChannel.LEDChannelB);  //Red
                canifier.setLEDOutput(0 * dimmer, CANifier.LEDChannel.LEDChannelC);  //Blue
                break;
            case -1:
                canifier.setLEDOutput(0.0, CANifier.LEDChannel.LEDChannelA);  //Green
                canifier.setLEDOutput(0.0, CANifier.LEDChannel.LEDChannelB);  //Red
                canifier.setLEDOutput(0.0, CANifier.LEDChannel.LEDChannelC);  //Blue
                break;
        }
    }

    @Override
    public void setRobotState(final RobotState robotState) {
        this.robotState = robotState;
    }

    @Override
    public void updateState() {
        if (POSTFinished) {
            if (robotState.isRobotEnabled()) {
                if (robotState.isLEDFlash()) {
                    if (flashDelayTimer < (flashTime * dutyCycle)) {
                        this.setLEDcolor(robotState.getLEDColorState());
                    } else {
                        this.setLEDcolor(-1);
                    }
                    if (flashDelayTimer >= flashTime) {
                        flashDelayTimer = 0;
                    }
                    flashDelayTimer++;
                } else {
                    this.setLEDcolor(robotState.getLEDColorState());
                }
            } else {
                this.setLEDcolor(4);
            }
            if (powerPannel.getVoltage() < 9.5) {
                dimmer = (powerPannel.getVoltage() - 7.3) / 2.2;
                if (dimmer < 0) dimmer = 0;
            }
            //if(dimmer > 0.40) dimmer = 0.40;
        } else {
            POSTRoutine();
        }

    }

    @Override
    public void zeroSensor() {
        // TODO Auto-generated method stub
        POSTFinished = false;
        StartState = 0;
        POSTTimer = 0;
    }

    @Override
    public void setSensorValue(Integer value) {
        // TODO Auto-generated method stub

    }

    @Override
    public void performAction(RobotCommandProvider commander, RobotState robotState) {
        // TODO Auto-generated method stub

    }


}