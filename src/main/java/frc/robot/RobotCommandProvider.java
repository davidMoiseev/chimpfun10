package frc.robot;

public abstract class RobotCommandProvider {
    private BallSupervisor.BallSupervisorState ballSupervisorState;
    private BallSupervisor.hoodPos hoodPosition;

    public abstract double getDriveCommand();
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

    public abstract double getTurnCommand();
    public abstract void chooseBallCommand();
}