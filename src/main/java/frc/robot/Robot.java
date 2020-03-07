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
   private Climber climber;
   private RobotCommandProvider commander;
   private AutoCommandProvider autoCommander;
   private BallSupervisor ballSupervisor;
   private HotController operator;
   private LEDController lEDController;
   private Arm arm;
   private Limelight limelite;
  private int disabledCounter = 0;

  @Override
  public void robotInit() {
    
    driver = new HotController(0, false);
    operator = new HotController(1, false);
    robotState = new RobotState();
    climber = new Climber(robotState);
    limelite = new Limelight(robotState);    
    drivetrain = new DriveTrain(robotState);
    commander = new TeleopCommandProvider(driver,operator,robotState);
    pigeon = new Pigeon(robotState);
    ballSupervisor = new BallSupervisor(robotState);
    lEDController = new LEDController(robotState);
    arm = new Arm(robotState);
    drivetrain.zeroSensor();
    climber.zeroSensor();
    pigeon.zeroSensor();
    lEDController.zeroSensor();
    ballSupervisor.zeroSensor();
  }


  @Override
  public void robotPeriodic() {
    drivetrain.updateState();
    pigeon.updateState();
    arm.updateState();
    limelite.updateState();
    lEDController.updateState();
    arm.updateState();
    ballSupervisor.updateState();
    climber.updateState();
  }

  @Override
  public void disabledPeriodic() {
    disabledCounter++;
    if( disabledCounter > 300){
      drivetrain.setBrake(false);
    }
    robotState.setRobotEnabled(false);
  }

  @Override
  public void disabledInit() {
    arm.setArmReset(true);
        
  }

  @Override
  public void autonomousInit() {
    HotLogger.Setup("theta", "auto_step_drive", "rightDistance", "leftDistance","autoTime", "left motor output", "right motor output",
    "drive ticks left", "drive ticks right", "ball inventory", "BallState", "carousel output", "conveyor output", "MasterIndicator State", "shooter speed",
    "Battery Voltage", "Current Current Draw", "has arm reset","Target Speed","Shooter PowerOutput", "arm commanded", "arm current");
    drivetrain.setBrake(true);
    pigeon.zeroSensor();
    arm.autoInitArmAngle();
    robotState.resetRobotState();
    autoCommander = new AutoCommandProvider(robotState);
    drivetrain.zeroPath();
    drivetrain.zeroActuators();
    drivetrain.zeroSensor();
    robotState.setRobotEnabled(true);
  }

  @Override
  public void autonomousPeriodic() {
    autoCommander.updateAutoRoutine();
    autoCommander.chooseBallCommand();
    drivetrain.performAction(autoCommander, robotState);
    ballSupervisor.performAction(autoCommander, robotState);
    arm.performAction(autoCommander, robotState);
  }

  @Override
  public void teleopInit() {
    HotLogger.Setup("theta", "auto_step_drive", "rightDistance", "leftDistance","autoTime", "left motor output", "right motor output",
    "drive ticks left", "drive ticks right", "ball inventory", "BallState", "carousel output", "conveyor output", "MasterIndicator State", "shooter speed",
    "Battery Voltage", "Current Current Draw", "has arm reset","Target Speed","Shooter PowerOutput", "arm commanded", "arm current");
    drivetrain.setBrake(true);
    //ballSupervisor.zeroSensor();
    robotState.setRobotEnabled(true);
  }

  @Override
  public void teleopPeriodic() {
    drivetrain.performAction(commander, robotState);
    commander.chooseBallCommand();
    climber.performAction(commander, robotState);
    ballSupervisor.performAction(commander, robotState);
    arm.performAction(commander, robotState);
    commander.setManualMode();
    commander.setLowPowerMode();
  }
  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    commander.lockManualMode(true);
    commander.chooseBallCommand();
    ballSupervisor.performAction(commander, robotState);
    arm.performAction(commander, robotState);
  }

}
