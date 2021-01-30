package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import org.hotutilites.hotInterfaces.IHotSensor;

public class Pigeon implements IHotSensor<RobotState, Double> {
    PigeonIMU pigeonIMU;
    private RobotState robotState;

    public Pigeon(RobotState robotState) {
        pigeonIMU = new PigeonIMU(Calibrations.CAN_ID.pigeon);
        this.robotState = robotState;
    }

    @Override
    public void updateState() {
        double[] ypr_deg = {0, 0, 0};
        pigeonIMU.getYawPitchRoll(ypr_deg);
        robotState.setTheta(ypr_deg[0]);
        robotState.setRoll(ypr_deg[2]);
    }

    public void setSensorValue(double value) {
        pigeonIMU.setYaw(value, 100);
    }

    @Override
    public void zeroSensor() {
        pigeonIMU.setYaw(0, 100);
    }

    @Override
    public void setSensorValue(Double vaule) {
        robotState.setTheta(vaule);
    }

    @Override
    public void setRobotState(RobotState robotState) {
        this.robotState = robotState;
    }

}