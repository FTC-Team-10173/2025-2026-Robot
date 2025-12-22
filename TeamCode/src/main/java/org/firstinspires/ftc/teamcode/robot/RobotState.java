package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.robot.subsystems.LED;

public class RobotState {
    public enum State {
        IDLE,
        INTAKING,
        SHOOTING,
        ALIGNING
    }

    private State currentState;
    DriverControls controls;

    public RobotState(DriverControls controls) {
        this.currentState = State.IDLE;
        this.controls = controls;
    }

    public RobotState() {
        this.currentState = State.IDLE;
    }

    /**
     * Update the current robot state based on control inputs
     */
    public void periodic() {
        State previousState = currentState;

        // Determine new state based on driver inputs
        if (controls.spinShooterPressed()) {
            currentState = State.SHOOTING;
        } else if (controls.intakePower() > 0.1 || controls.fullIntakePressed()) {
            currentState = State.INTAKING;
        } else if (controls.lockDrivePressed()) {
            currentState = State.ALIGNING;
        } else {
            currentState = State.IDLE;
        }

        // Handle state transitions (if needed in future)
        if (previousState != currentState) {
            onStateChange(previousState, currentState);
        }
    }

    /**
     * Get the current robot state
     */
    public State get() {
        return currentState;
    }

    /**
     * Manually set the robot state
     */
    public void set(State state) {
        if (currentState != state) {
            State previousState = currentState;
            currentState = state;
            onStateChange(previousState, state);
        }
    }

    /**
     * Check if robot is in a specific state
     */
    public boolean is(State state) {
        return currentState == state;
    }

    /**
     * Called whenever the state changes
     * Can be used to trigger one-time actions on state transitions
     */
    private void onStateChange(State from, State to) {
        // Currently no special transition logic
        // Could add: rumble controller, play sounds, etc.
    }

    /**
     * String representation of the current state
     */
    @NonNull
    @Override
    public String toString() {
        return currentState.toString();
    }
}