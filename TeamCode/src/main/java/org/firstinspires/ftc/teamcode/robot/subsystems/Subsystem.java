package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Logger;

public interface Subsystem {
    void periodic();


    default void init() {}

    default void stop() {}

    default void updateTelemetry(Telemetry telemetry, TelemetryPacket packet, Logger logger) {}

    default String getName() {
        return this.getClass().getSimpleName();
    }


    default boolean isHealthy() {
        return true;
    }
}