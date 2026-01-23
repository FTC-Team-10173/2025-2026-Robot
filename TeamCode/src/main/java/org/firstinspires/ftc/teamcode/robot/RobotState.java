package org.firstinspires.ftc.teamcode.robot;

public class RobotState {
    public enum State {
        IDLE,
        INTAKING,
        SHOOTING_READY,
        SPINNING_UP,
        ALIGNING
    }

    private State currentState = State.IDLE;
    private boolean shooterReady = false;
    private boolean shooterRunning = false;
    private boolean intakeRunning = false;

    public RobotState() {}

    public void set(State state) {
        this.currentState = state;
    }

    public State get() {
        return currentState;
    }

    public boolean is(State state) {
        return currentState == state;
    }

    public void setShooterReady(boolean ready) {
        this.shooterReady = ready;
    }

    public boolean isShooterReady() {
        return shooterReady;
    }

    public void setShooterRunning(boolean running) {
        this.shooterRunning = running;
    }

    public boolean isShooterRunning() {
        return shooterRunning;
    }

    public void setIntakeRunning(boolean running) {
        this.intakeRunning = running;
    }

    public boolean isIntakeRunning() {
        return intakeRunning;
    }

    @Override
    public String toString() {
        return currentState.toString();
    }
}