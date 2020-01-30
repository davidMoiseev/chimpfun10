/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import org.hotutilites.hotcontroller.HotController;
import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

   private RobotState robotState;
   private DriveTrain drivetrain;
   private Pigeon pigeon;
   private HotController driver;
   private RobotCommandProvider commander;

  @Override
  public void robotInit() {
    HotLogger.Setup("theta","Drive_Distance_Right","Drive_Distance_Left");
    driver = new HotController(0, false);
    robotState = new RobotState();    
    drivetrain = new DriveTrain(robotState);
    commander = new TeleopCommandProvider(driver);
    pigeon = new Pigeon(robotState);

    drivetrain.zeroSensor();
    pigeon.zeroSensor();
  }

  @Override
  public void robotPeriodic() {
    drivetrain.updateState();
    pigeon.updateState();
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    drivetrain.performAction(commander, robotState);
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
