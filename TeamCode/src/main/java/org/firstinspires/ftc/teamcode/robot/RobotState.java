package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

public class RobotState {
    public enum State {
        IDLE,
        INTAKING,
        SHOOTING_READY,
        SPINNING_UP,
        ALIGNING
    }

    private State currentState;
    DriverControls controls;

    public Boolean shooterReady = false;

    public RobotState(DriverControls controls) {
        this.currentState = State.IDLE;
        this.controls = controls;
    }

    public RobotState() {
        this.currentState = State.IDLE;
    }

    public void periodic() {
        // Determine new state based on driver inputs and shooter status
        if (controls.spinShooterPressed()) {
            if (shooterReady) {
                currentState = State.SHOOTING_READY;
            } else {
                currentState = State.SPINNING_UP;
            }
        } else if (controls.intakePower() > 0.1 || controls.fullIntakePressed()) {
            currentState = State.INTAKING;
        } else if (controls.lockDrivePressed()) {
            currentState = State.ALIGNING;
        } else {
            currentState = State.IDLE;
        }
    }

    public State get() {
        return currentState;
    }

    public void set(State state) {
        if (currentState != state) {
            currentState = state;
        }
    }
    public boolean is(State state) {
        return currentState == state;
    }

    @NonNull
    @Override
    public String toString() {
        return currentState.toString();
    }
}