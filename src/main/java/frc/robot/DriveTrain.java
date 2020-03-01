package frc.robot;

import edu.wpi.first.wpilibj.SpeedController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import org.hotutilites.hotInterfaces.IHotSensedActuator;

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
    private double steeringAdjust, distanceAdjust;
    private double headingError;
    private double currentDistanceFromTarget, desiredDistanceFromTarget, distanceError;

    public DriveTrain(RobotState robotState) {

      
        driveRightLeader = new TalonFX(Calibrations.CAN_ID.driveRight1);
        driveRightFollower = new TalonFX(Calibrations.CAN_ID.driveRight2);
        driveLeftLeader = new TalonFX(Calibrations.CAN_ID.driveLeft1);
        driveLeftFollower = new TalonFX(Calibrations.CAN_ID.driveLeft2);

        driveRightLeader.configFactoryDefault();
        driveLeftLeader.configFactoryDefault();

        driveRightLeader.configNeutralDeadband(0.001);
        driveLeftLeader.configNeutralDeadband(0.001);

        driveRightLeader.setInverted(true);
        driveRightFollower.setInverted(TalonFXInvertType.FollowMaster);

        driveLeftFollower.follow(driveLeftLeader);
        driveRightFollower.follow(driveRightLeader);

        driveRightLeader.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 , 30);
        driveLeftLeader.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 , 30);

        driveRightLeader.configNominalOutputForward(0,30);
		driveRightLeader.configNominalOutputReverse(0, 30);
		driveRightLeader.configPeakOutputForward(1, 20);
        driveRightLeader.configPeakOutputReverse(-1, 30);
        
    
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

        pidControllerSteering = new PIDController(Calibrations.Vision.kP, Calibrations.Vision.kI, Calibrations.Vision.kD);
        pidControllerDistance = new PIDController(Calibrations.Vision.kP, Calibrations.Vision.kI, Calibrations.Vision.kD);

        trajFollower = new TrajectoryFollower(0);
        followingPath = false;
        

        this.robotState = robotState;
    }

    @Override
    public void updateState() {
 
        robotState.setDriveDistanceRight(driveRightLeader.getSelectedSensorPosition()/Calibrations.DRIVE_CONSTANTS.ticksPerMeter);
        robotState.setDriveDistanceLeft(driveLeftLeader.getSelectedSensorPosition()/Calibrations.DRIVE_CONSTANTS.ticksPerMeter);
        SmartDashboard.putNumber("driveticksleft", driveRightLeader.getSelectedSensorPosition());
        SmartDashboard.putNumber("driveticksright", driveLeftLeader.getSelectedSensorPosition());
      

        robotState.setDriveVelocityLeft((driveLeftLeader.getSelectedSensorVelocity() *10 / Calibrations.DRIVE_CONSTANTS.ticksPerMeter));  //unit conversions- ticks/ms to m/s
        robotState.setDriveVelocityRight(driveRightLeader.getSelectedSensorVelocity()*10 / Calibrations.DRIVE_CONSTANTS.ticksPerMeter);
    }

    @Override
    public void performAction(RobotCommandProvider commander, RobotState robotState) {
        if(commander.isLowPowerMode()){
            driveLeftLeader.configPeakOutputForward(0.25);
            driveRightLeader.configPeakOutputForward(0.25);
            driveLeftLeader.configPeakOutputReverse(-0.1);
            driveRightLeader.configPeakOutputReverse(-0.1);
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
        

            trajFollower.update(robotState.getTheta(), robotState.getDriveDistanceLeft(), robotState.getDriveDistanceRight());

            leftOutput = (trajFollower.leftOutput * Calibrations.DRIVE_CONSTANTS.ticksPerMeter)/ 10 ;
            rightOutput = (trajFollower.rightOutput * Calibrations.DRIVE_CONSTANTS.ticksPerMeter)/ 10;
            driveLeftLeader.set((ControlMode.Velocity), leftOutput, DemandType.ArbitraryFeedForward, ((leftffController.calculate(trajFollower.leftOutput))/driveLeftLeader.getBusVoltage()));
            driveRightLeader.set((ControlMode.Velocity), rightOutput, DemandType.ArbitraryFeedForward,((rightffController.calculate(trajFollower.rightOutput))/driveRightLeader.getBusVoltage()));
            
            SmartDashboard.putNumber("following path", followingPathCount);
            SmartDashboard.putNumber("following path", followingPathCount);
            SmartDashboard.putNumber("left Output", leftOutput);
            SmartDashboard.putNumber("right Output", rightOutput);
            followingPathCount++;
        }

        else{
            driveRightLeader.set(ControlMode.PercentOutput, commander.getDriveCommand() + commander.getTurnCommand());
            driveLeftLeader.set(ControlMode.PercentOutput, commander.getDriveCommand() - commander.getTurnCommand());
        }
        
        robotState.setTrajectoryComplete(trajFollower.isTrajFinished());
    }
  

    public void zeroActuators() {
        driveRightLeader.set(ControlMode.PercentOutput, 0);
        driveRightFollower.set(ControlMode.PercentOutput, 0);
        driveLeftLeader.set(ControlMode.PercentOutput, 0);
        driveLeftFollower.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void zeroSensor() {
        driveRightLeader.setSelectedSensorPosition(0);
        driveRightFollower.setSelectedSensorPosition(0);
        driveLeftLeader.setSelectedSensorPosition(0);
        driveLeftFollower.setSelectedSensorPosition(0);
        
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
 


}