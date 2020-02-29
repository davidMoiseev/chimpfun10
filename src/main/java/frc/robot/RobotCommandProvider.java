package frc.robot;

import org.hotutilites.hotcontroller.HotController;

import frc.robot.TrajectoryFollower.PathNames;

public abstract class RobotCommandProvider {
    private BallSupervisor.BallSupervisorState ballSupervisorState;
    private BallSupervisor.hoodPos hoodPosition;
    private Arm.ArmPositions ArmPosition;

    public abstract double getDriveCommand();

    public Arm.ArmPositions getArmPosition() {
        return ArmPosition;
    }

    public void setArmPosition(Arm.ArmPositions armPosition) {
        this.ArmPosition = armPosition;
    }

    public abstract boolean getAimingEnabled();
    public abstract boolean getRangeEnabled();

    public BallSupervisor.hoodPos getHoodPosition() {
        return hoodPosition;
    }

    public void setHoodPosition(BallSupervisor.hoodPos hoodPosition) {
        this.hoodPosition = hoodPosition;
    }

    public BallSupervisor.BallSupervisorState getBallSupervisorState() {
        return ballSupervisorState;
    }

    public void setBallSupervisorState(BallSupervisor.BallSupervisorState ballSupervisorState) {
        this.ballSupervisorState = ballSupervisorState;
    }

    public abstract void setManualMode();
    public abstract HotController getOperator();
    public abstract double getTurnCommand();
    public abstract void chooseBallCommand();
    public abstract PathNames getPathName();
    public abstract double getArmOutput();
    public abstract boolean getPathFollowingCommand();
}