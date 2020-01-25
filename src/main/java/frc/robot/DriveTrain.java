package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.hotutilites.hotInterfaces.IHotSensedActuator;
import org.hotutilites.hotInterfaces.IRobotState;

public class DriveTrain implements IHotSensedActuator {
    private TalonFX driveRightLeader;
    private TalonFX driveRightFollower;
    private TalonFX driveLeftLeader;
    private TalonFX driveLeftFollower;

    public DriveTrain() {
        driveRightLeader = new TalonFX(Calibrations.CAN_ID.driveRight1);
        driveRightFollower = new TalonFX(Calibrations.CAN_ID.driveRight2);
        driveLeftLeader = new TalonFX(Calibrations.CAN_ID.driveLeft1);
        driveLeftFollower = new TalonFX(Calibrations.CAN_ID.driveLeft2);

        driveRightLeader.setInverted(true);
        driveRightFollower.setInverted(TalonFXInvertType.FollowMaster);

        driveLeftFollower.follow(driveLeftLeader);
        driveRightFollower.follow(driveRightLeader);
    }

    @Override
    public void updateState(IRobotState IrobotState) {
        RobotState robotState = (RobotState)IrobotState;
        robotState.setDriveDistanceRight(driveRightLeader.getSelectedSensorPosition());
        robotState.setDriveDistanceLeft(driveLeftLeader.getSelectedSensorPosition());
    }

    @Override
    public void preformAction(RobotCommandProvider commander, IRobotState robotState) {
        driveRightLeader.set(ControlMode.PercentOutput,commander.getDriveCommand() + commander.getTurnCommand());
        driveLeftLeader.set(ControlMode.PercentOutput,commander.getDriveCommand() - commander.getTurnCommand());
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
}