package frc.robot;
import javax.swing.text.Position;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import org.hotutilites.hotInterfaces.IHotSensedActuator;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class Climber implements IHotSensedActuator<RobotState, RobotCommandProvider, Integer> {
    RobotState robotState;
    climberController leftClimber;
    climberController rightClimber;

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
        public climberController(int canID) {
            motor = new TalonFX(canID);
            motor.configFactoryDefault();
            motor.selectProfileSlot(0, 0);
            motor.configNominalOutputForward(0);
            motor.configNominalOutputReverse(0);
            motor.configPeakOutputForward(1);
            motor.configPeakOutputReverse(-1);
            motor.setNeutralMode(NeutralMode.Brake);
        }
        public int getStatusColor() {
            return statusColor;
        }
        public void setPower(double power) {
            this.overridePower = power;
            motor.set(ControlMode.PercentOutput, power);
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
            motor.setSelectedSensorPosition(0);
            inAuto = false;
            released = false;
            enabled = false;
        }
        public double getCurrentHeight(){
            return position;
        }
    }
    

    public Climber(RobotState state){
        this.robotState = state;
        leftClimber = new climberController(Calibrations.CAN_ID.leftClimber);
        rightClimber = new climberController(Calibrations.CAN_ID.rightClimber);
    }
    
    
    public void performAction(RobotCommandProvider commander, RobotState state){  
        
        
        if(commander.isLeftClimberActivate() || commander.isRightClimberActivate()){
       
            leftClimber.setPower(-commander.getLeftClimberDelta());
            rightClimber.setPower(-commander.getRightClimberDelta());
        }else{
           
            leftClimber.setPower(0);
            rightClimber.setPower(0);
        }
    }
    @Override
    public void updateState() {
        SmartDashboard.putNumber("RobotRoll", (int)robotState.getRoll());
        SmartDashboard.putNumber("LeftClimberStatus", leftClimber.statusColor);
        SmartDashboard.putNumber("RightClimberStatus", rightClimber.statusColor);
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