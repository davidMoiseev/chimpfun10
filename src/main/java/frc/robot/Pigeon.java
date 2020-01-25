package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import org.hotutilites.hotInterfaces.IHotSensor;
import org.hotutilites.hotInterfaces.IRobotState;
import org.hotutilites.hotlogger.HotLogger;

public class Pigeon implements IHotSensor
{
    PigeonIMU pigeonIMU;

public Pigeon() {
    pigeonIMU = new PigeonIMU(Calibrations.CAN_ID.pigeon);
}

@Override
public void updateState(IRobotState robotState) {
    double[] ypr_deg = null;
    pigeonIMU.getYawPitchRoll(ypr_deg);
    ((RobotState)robotState).setTheta(ypr_deg[0]);
}

public void setSensorValue(double value) {

    pigeonIMU.setYaw(value, 100);
}

    @Override
    public void zeroSensor() {
        pigeonIMU.setYaw(0, 100);
    }

}