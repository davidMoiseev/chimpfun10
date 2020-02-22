package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.hotutilites.hotInterfaces.IHotSensedActuator;

import edu.wpi.first.wpilibj.controller.PIDController;

public class DriveTrain implements IHotSensedActuator<RobotState, RobotCommandProvider, Integer > {
    private TalonFX driveRightLeader;
    private TalonFX driveRightFollower;
    private TalonFX driveLeftLeader;
    private TalonFX driveLeftFollower;
    private RobotState robotState;

    private PIDController pidControllerSteering, pidControllerDistance;
    private double steeringAdjust, distanceAdjust;
    private double headingError;
    private double currentDistanceFromTarget, desiredDistanceFromTarget, distanceError;

    public DriveTrain(RobotState robotState) {
        driveRightLeader = new TalonFX(Calibrations.CAN_ID.driveRight1);
        driveRightFollower = new TalonFX(Calibrations.CAN_ID.driveRight2);
        driveLeftLeader = new TalonFX(Calibrations.CAN_ID.driveLeft1);
        driveLeftFollower = new TalonFX(Calibrations.CAN_ID.driveLeft2);

        driveRightLeader.setInverted(true);
        driveRightFollower.setInverted(TalonFXInvertType.FollowMaster);

        driveLeftFollower.follow(driveLeftLeader);
        driveRightFollower.follow(driveRightLeader);

        this.robotState = robotState;

        pidControllerSteering = new PIDController(Calibrations.Vision.kP, Calibrations.Vision.kI, Calibrations.Vision.kD);
        pidControllerDistance = new PIDController(Calibrations.Vision.kP, Calibrations.Vision.kI, Calibrations.Vision.kD);

    }

    @Override
    public void updateState() {
        robotState.setDriveDistanceRight(driveRightLeader.getSelectedSensorPosition());
        robotState.setDriveDistanceLeft(driveLeftLeader.getSelectedSensorPosition());
    }

    @Override
    public void performAction(RobotCommandProvider commander, RobotState robotState) {
        driveRightLeader.set(ControlMode.PercentOutput,-commander.getDriveCommand() + commander.getTurnCommand());
        driveLeftLeader.set(ControlMode.PercentOutput,-commander.getDriveCommand() - commander.getTurnCommand());
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
    if (commander.getRangeEnabled()){
        currentDistanceFromTarget = robotState.getDistanceFromTarget();
        desiredDistanceFromTarget = 2.2;
        distanceError = desiredDistanceFromTarget - currentDistanceFromTarget;
        distanceAdjust = pidControllerDistance.calculate(distanceError);
        driveLeftLeader.set(ControlMode.PercentOutput, distanceAdjust);
        driveRightLeader.set(ControlMode.PercentOutput, distanceAdjust);
    }
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
}