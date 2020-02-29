package frc.robot;

import org.hotutilites.hotInterfaces.IHotSensedActuator;
public class Climber implements IHotSensedActuator<RobotState, RobotCommandProvider, Integer > {
    RobotState state;
    
    public Climber(RobotState state){
        
    }
    
    public void performAction(RobotCommandProvider commander, RobotState state){  
        
        }
        
       
    
    @Override
    public void updateState() {
        
    }
    @Override
    public void zeroSensor() {
        
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