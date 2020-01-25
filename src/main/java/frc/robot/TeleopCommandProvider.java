package frc.robot;

import org.hotutilites.hotcontroller.HotController;

public class TeleopCommandProvider extends RobotCommandProvider {

    private HotController driver;

    public TeleopCommandProvider(HotController driver) {
        this.setDriver(driver);
    }

    public double getDriveCommand() {
        return driver.getStickLY();
    }

    public double getTurnCommand() {
        return driver.getStickRX();
    }

    private void setDriver(HotController driver) {
        this.driver = driver;
    }

}