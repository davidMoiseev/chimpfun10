/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.hotutilites.hotcontroller.HotController;
import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.BallSupervisor.BallSupervisorState;


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
   private BallSupervisor ballSupervisor;
   private HotController operator;
   private LEDController lEDController;
   private Arm arm;
   private Limelight limelite;

  @Override
  public void robotInit() {
    HotLogger.Setup("theta","Drive_Distance_Right","Drive_Distance_Left");
    driver = new HotController(0, false);
    operator = new HotController(1, false);
    robotState = new RobotState();
    limelite = new Limelight(robotState);    
    drivetrain = new DriveTrain(robotState);
    commander = new TeleopCommandProvider(driver,operator,robotState);
    pigeon = new Pigeon(robotState);
    ballSupervisor = new BallSupervisor(robotState);
    lEDController = new LEDController(robotState);
    arm = new Arm();
    drivetrain.zeroSensor();
    pigeon.zeroSensor();
    ballSupervisor.zeroSensor();
  }


  @Override
  public void robotPeriodic() {
    ballSupervisor.updateState();
    drivetrain.updateState();
    pigeon.updateState();
    limelite.updateState();
    lEDController.updateState();
  }

  @Override
  public void disabledPeriodic() {
    robotState.setRobotEnabled(false);
  }

  @Override
  public void autonomousInit() {
    robotState.setRobotEnabled(true);
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    ballSupervisor.zeroSensor();
    robotState.setRobotEnabled(true);
  }

  @Override
  public void teleopPeriodic() {
    drivetrain.performAction(commander, robotState);
    arm.performAction(commander, robotState);
    commander.chooseBallCommand();
    ballSupervisor.performAction(commander, robotState);
    commander.setManualMode();
  }
  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  
  }

}
