package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Interface for all robot subsystems
 * Provides a common structure and lifecycle methods
 */
public interface Subsystem {
    /**
     * Called periodically during the main robot loop
     * This is where subsystems update their state based on inputs
     */
    void periodic();

    /**
     * Called once when the subsystem is first initialized
     * Optional override - default implementation does nothing
     */
    default void init() {
        // Default: do nothing
    }

    /**
     * Called when the robot is stopped or disabled
     * Use this to safely stop motors and return to a safe state
     */
    default void stop() {
        // Default: do nothing
    }

    /**
     * Add telemetry data for this subsystem
     * Optional override - default implementation does nothing
     *
     * @param telemetry The telemetry object to add data to
     */
    default void updateTelemetry(Telemetry telemetry, TelemetryPacket packet) {
        // Default: do nothing
    }

    /**
     * Get the name of this subsystem for logging and telemetry
     * Optional override - defaults to class simple name
     *
     * @return The name of this subsystem
     */
    default String getName() {
        return this.getClass().getSimpleName();
    }

    /**
     * Check if this subsystem is in a safe state
     * Optional override - default returns true
     *
     * @return true if subsystem is safe/healthy, false otherwise
     */
    default boolean isHealthy() {
        return true;
    }
}