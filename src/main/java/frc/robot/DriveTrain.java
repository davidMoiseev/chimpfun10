package frc.robot;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import org.hotutilites.hotInterfaces.IHotSensedActuator;
import org.hotutilites.hotlogger.HotLogger;

public class DriveTrain implements IHotSensedActuator<RobotState, RobotCommandProvider, Integer > {



    private TalonFX driveRightLeader;
    private TalonFX driveRightFollower;
    private TalonFX driveLeftLeader;
    private TalonFX driveLeftFollower;
    private RobotState robotState;
    
    //private SimpleMotorFeedforward autoDriveFF = new SimpleMotorFeedforward(Calibrations.AUTO_CONTROLLERS.driveFFks, Calibrations.AUTO_CONTROLLERS.driveFFkv);
    //private PIDController autoDriveVelocityPID = new PIDController(Calibrations.AUTO_CONTROLLERS.velocityPIDkp, Calibrations.AUTO_CONTROLLERS.velocityPIDki, Calibrations.AUTO_CONTROLLERS.velocityPIDkd);

    public double driveVelocityLeft;
    public double driveVelocityRight;

    public Orchestra musicPlayer;

    private boolean followingPath = false;
    private boolean previouslyFollowingPath = false;

    TrajectoryFollower trajFollower;
    private int startedPathCount = 1;
    private int followingPathCount = 1;
    private double leftOutput;
    private double rightOutput;

    private SimpleMotorFeedforward leftffController = new SimpleMotorFeedforward(Calibrations.AUTO_CONTROLLERS.ffkS, Calibrations.AUTO_CONTROLLERS.ffkV, Calibrations.AUTO_CONTROLLERS.ffkA);
    private SimpleMotorFeedforward rightffController = new SimpleMotorFeedforward(Calibrations.AUTO_CONTROLLERS.ffkS, Calibrations.AUTO_CONTROLLERS.ffkV, Calibrations.AUTO_CONTROLLERS.ffkA);

    private PIDController pidControllerSteering, pidControllerDistance;
    private PIDController lDrivePID, rDrivePID;
    private double steeringAdjust, distanceAdjust;
    private double headingError;
    private double currentDistanceFromTarget, desiredDistanceFromTarget, distanceError;
    private double leftFF;
    private double rightFF;
    private double driveDistanceRight = 0;
    private double driveDistanceLeft = 0;
    private double autoDriveStraightRight = 0;
    private double autoDriveStraightLeft = 0;
    private double distanceDifference = 0;;



    public DriveTrain(RobotState robotState) {

      
        driveRightLeader = new TalonFX(Calibrations.CAN_ID.driveRight1);
        driveRightFollower = new TalonFX(Calibrations.CAN_ID.driveRight2);
        driveLeftLeader = new TalonFX(Calibrations.CAN_ID.driveLeft1);
        driveLeftFollower = new TalonFX(Calibrations.CAN_ID.driveLeft2);

        driveRightLeader.configFactoryDefault();
        driveLeftLeader.configFactoryDefault();
        driveRightFollower.configFactoryDefault();
        driveLeftFollower.configFactoryDefault();

        driveRightLeader.setNeutralMode(NeutralMode.Brake);
        driveRightFollower.setNeutralMode(NeutralMode.Brake);
        driveLeftLeader.setNeutralMode(NeutralMode.Brake);
        driveLeftFollower.setNeutralMode(NeutralMode.Brake);

        driveRightLeader.configNeutralDeadband(0.001);
        driveLeftLeader.configNeutralDeadband(0.001);

        driveRightLeader.setInverted(true);
        driveRightFollower.setInverted(TalonFXInvertType.FollowMaster);

        driveLeftFollower.follow(driveLeftLeader);
        driveRightFollower.follow(driveRightLeader);

        driveRightLeader.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 , 30);
        driveLeftLeader.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 , 30);
        driveRightFollower.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 , 30);
        driveLeftFollower.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 , 30);

        driveRightLeader.configNominalOutputForward(0,30);
		driveRightLeader.configNominalOutputReverse(0, 30);
		driveRightLeader.configPeakOutputForward(1, 20);
        driveRightLeader.configPeakOutputReverse(-1, 30);

        // driveRightLeader.configVoltageCompSaturation(12);
        // driveLeftLeader.configVoltageCompSaturation(12);
        // driveRightLeader.enableVoltageCompensation(true);
        // driveLeftLeader.enableVoltageCompensation(true);
    
        
    
		driveRightLeader.config_kP(0, Calibrations.AUTO_CONTROLLERS.velocityPIDkp, 30);
		driveRightLeader.config_kI(0, Calibrations.AUTO_CONTROLLERS.velocityPIDki, 30);
		driveRightLeader.config_kD(0, Calibrations.AUTO_CONTROLLERS.velocityPIDkd, 30);
        driveRightLeader.config_kF(0, Calibrations.AUTO_CONTROLLERS.velocityPIDkf, 30);

        driveLeftLeader.configNominalOutputForward(0,30);
		driveLeftLeader.configNominalOutputReverse(0, 30);
		driveLeftLeader.configPeakOutputForward(1, 20);
        driveLeftLeader.configPeakOutputReverse(-1, 30);
        
    
		driveLeftLeader.config_kP(0, Calibrations.AUTO_CONTROLLERS.velocityPIDkp, 30);
		driveLeftLeader.config_kI(0, Calibrations.AUTO_CONTROLLERS.velocityPIDki, 30);
		driveLeftLeader.config_kD(0, Calibrations.AUTO_CONTROLLERS.velocityPIDkd, 30);
        driveLeftLeader.config_kF(0, Calibrations.AUTO_CONTROLLERS.velocityPIDkf, 30);

        lDrivePID = new PIDController(Calibrations.AUTO_CONTROLLERS.voltagekP, Calibrations.AUTO_CONTROLLERS.voltagekI, Calibrations.AUTO_CONTROLLERS.voltagekD);
        rDrivePID = new PIDController(Calibrations.AUTO_CONTROLLERS.voltagekP, Calibrations.AUTO_CONTROLLERS.voltagekI, Calibrations.AUTO_CONTROLLERS.voltagekD);

        pidControllerSteering = new PIDController(Calibrations.Vision.kP, Calibrations.Vision.kI, Calibrations.Vision.kD);
        pidControllerDistance = new PIDController(Calibrations.Vision.kP, Calibrations.Vision.kI, Calibrations.Vision.kD);

  

        trajFollower = new TrajectoryFollower(0);
        followingPath = false;

        this.robotState = robotState;
    }

    @Override
    public void updateState() {

        driveDistanceRight = (driveRightLeader.getSelectedSensorPosition()/Calibrations.DRIVE_CONSTANTS.ticksPerMeter);
        driveDistanceLeft = (driveLeftLeader.getSelectedSensorPosition()/Calibrations.DRIVE_CONSTANTS.ticksPerMeter);

        robotState.setDriveDistanceRight(driveDistanceRight);
        robotState.setDriveDistanceLeft(driveDistanceLeft);
        SmartDashboard.putNumber("drive distance left", driveDistanceLeft);
        SmartDashboard.putNumber("drive distance right", driveDistanceRight);


        robotState.setDriveVelocityLeft((driveLeftLeader.getSelectedSensorVelocity() *10 / Calibrations.DRIVE_CONSTANTS.ticksPerMeter));  //unit conversions- ticks/ms to m/s
        robotState.setDriveVelocityRight(driveRightLeader.getSelectedSensorVelocity()*10 / Calibrations.DRIVE_CONSTANTS.ticksPerMeter);
        
        HotLogger.Log("left drive current", driveLeftLeader.getStatorCurrent());
        HotLogger.Log("right drive current", driveRightLeader.getStatorCurrent());
    }

    @Override
    public void performAction(RobotCommandProvider commander, RobotState robotState) {

        if(commander.getEncodersReset()){
            zeroSensor();
        }

        if(commander.isLowPowerMode()){
            driveLeftLeader.configPeakOutputForward(0.05);
            driveRightLeader.configPeakOutputForward(0.05);
            driveLeftLeader.configPeakOutputReverse(-0.25);
            driveRightLeader.configPeakOutputReverse(-0.25);
        }else{
            driveLeftLeader.configPeakOutputForward(1);
            driveRightLeader.configPeakOutputForward(1);
            driveLeftLeader.configPeakOutputReverse(-1);
            driveRightLeader.configPeakOutputReverse(-1);
        }

    

        previouslyFollowingPath = followingPath;
        followingPath = commander.getPathFollowingCommand();

        if (commander.getAimingEnabled()){
            headingError = robotState.getLimelightXTheta();    
            if (Math.abs(headingError) > Calibrations.Vision.deadband) {
                steeringAdjust = pidControllerSteering.calculate(headingError);
            } else {
                steeringAdjust = 0;
            }
            driveLeftLeader.set(ControlMode.PercentOutput, steeringAdjust);
            driveRightLeader.set(ControlMode.PercentOutput, -steeringAdjust);
        }

        else if (commander.getRangeEnabled()){
            currentDistanceFromTarget = robotState.getDistanceFromTarget();
            desiredDistanceFromTarget = 6.6;
            distanceError = desiredDistanceFromTarget - currentDistanceFromTarget;
            distanceAdjust = pidControllerDistance.calculate(distanceError);
            driveLeftLeader.set(ControlMode.PercentOutput, -distanceAdjust);
            driveRightLeader.set(ControlMode.PercentOutput, -distanceAdjust);
        }
        else if (followingPath && !previouslyFollowingPath) {
            zeroActuators();
            zeroSensor();
            trajFollower.startTrajectory(commander.getPathName(), robotState.getTheta());
            SmartDashboard.putNumber("started path", startedPathCount);
            startedPathCount++;
        }
        else if (followingPath){
        
            trajFollower.update(robotState.getTheta(), driveDistanceLeft, driveDistanceRight);

            
            leftOutput = (1 * Calibrations.DRIVE_CONSTANTS.ticksPerMeter) * 10 ;
            rightOutput = (1 * Calibrations.DRIVE_CONSTANTS.ticksPerMeter) * 10;

            leftOutput = lDrivePID.calculate(driveDistanceLeft, trajFollower.leftOutput);
            rightOutput = rDrivePID.calculate(driveDistanceRight, trajFollower.rightOutput);
            leftFF = (leftffController.calculate(trajFollower.leftOutput));
            rightFF = (rightffController.calculate(trajFollower.rightOutput));

            driveLeftLeader.set((ControlMode.PercentOutput), (leftOutput + leftFF) / 12);
            driveRightLeader.set((ControlMode.PercentOutput), (rightOutput + rightFF)/ 12);

            SmartDashboard.putNumber("111 pid target vel", driveLeftLeader.getActiveTrajectoryVelocity());
            SmartDashboard.putNumber("111 motor act vel", driveLeftLeader.getSelectedSensorVelocity() *10 / Calibrations.DRIVE_CONSTANTS.ticksPerMeter);
       
         


            SmartDashboard.putNumber("following path", followingPathCount);
           
            SmartDashboard.putNumber("following path", followingPathCount);
            SmartDashboard.putNumber("left Output", leftOutput);
            SmartDashboard.putNumber("right Output", rightOutput);

            
            followingPathCount++;
        }
    
        else{
            double correction = 0;

            if (commander.getDriveYawCorrectionEnabled()) {
               double error = commander.getDriveYawCorrection() - robotState.getTheta();
               correction = error * Calibrations.DRIVE_CONSTANTS.YawCorrection_kP;
            } else  {
            correction = 0;
            }

            // if (commander.getDriveCommand() < 0) {
            //     correction = -correction;
            // }
        
            driveRightLeader.set(ControlMode.PercentOutput, commander.getDriveCommand() + commander.getTurnCommand() + correction);
            driveLeftLeader.set(ControlMode.PercentOutput, commander.getDriveCommand() - commander.getTurnCommand() - correction);

        }

        HotLogger.Log("left motor output", driveLeftLeader.getMotorOutputPercent());
        HotLogger.Log("right motor output", driveRightLeader.getMotorOutputPercent());
        // HotLogger.Log("path feedback output left", leftOutput);
        // HotLogger.Log("path feedback output right", rightOutput);
        // HotLogger.Log("path feedforward output left", leftFF);
        // HotLogger.Log("path feedforward output right", rightFF);
        HotLogger.Log("drive ticks left", driveLeftLeader.getSelectedSensorPosition());
         HotLogger.Log("drive ticks right", driveRightLeader.getSelectedSensorPosition());
            
        robotState.setTrajectoryComplete(trajFollower.isTrajFinished());
    }
  

    public void zeroActuators() {
        driveRightLeader.set(ControlMode.PercentOutput, 0);
        // driveRightFollower.set(ControlMode.PercentOutput, 0);
        driveLeftLeader.set(ControlMode.PercentOutput, 0);
        // driveLeftFollower.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void zeroSensor() {
        driveRightLeader.setSelectedSensorPosition(0,0,30);
        driveRightFollower.setSelectedSensorPosition(0,0,30);
        driveLeftLeader.setSelectedSensorPosition(0,0,30);
        driveLeftFollower.setSelectedSensorPosition(0,0,30);
        driveRightLeader.setSelectedSensorPosition(0,1,30);
        driveRightFollower.setSelectedSensorPosition(0,1,30);
        driveLeftLeader.setSelectedSensorPosition(0,1,30);
        driveLeftFollower.setSelectedSensorPosition(0,1,30);
        
    }

    @Override
    public void setRobotState(RobotState robotState) {
        this.robotState = robotState;
    }

    @Override
    public void setSensorValue(Integer value) {
        driveRightLeader.setSelectedSensorPosition(value);
        driveRightFollower.setSelectedSensorPosition(value);
        driveLeftLeader.setSelectedSensorPosition(value);
        driveLeftFollower.setSelectedSensorPosition(value);
    }

    public void setSensorValue(int left, int right) {
        driveRightLeader.setSelectedSensorPosition(right);
        driveRightFollower.setSelectedSensorPosition(right);
        driveLeftLeader.setSelectedSensorPosition(left);
        driveLeftFollower.setSelectedSensorPosition(left);
    }

    public void zeroPath(){ //change later, this is stupid
        followingPath = false;
    }
 
    public void setBrake(boolean brake){
        if(brake == true){
        driveRightLeader.setNeutralMode(NeutralMode.Brake);
        driveRightFollower.setNeutralMode(NeutralMode.Brake);
        driveLeftLeader.setNeutralMode(NeutralMode.Brake);
        driveLeftFollower.setNeutralMode(NeutralMode.Brake);
        }
        else{
            driveRightLeader.setNeutralMode(NeutralMode.Coast);
            driveRightFollower.setNeutralMode(NeutralMode.Coast);
            driveLeftLeader.setNeutralMode(NeutralMode.Coast);
            driveLeftFollower.setNeutralMode(NeutralMode.Coast);
        }
    }


}