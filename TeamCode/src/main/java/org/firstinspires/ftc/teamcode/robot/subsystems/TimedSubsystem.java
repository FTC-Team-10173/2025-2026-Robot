package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Logger;

public class TimedSubsystem implements Subsystem{
    private final Subsystem wrapped;
    private final ElapsedTime timer = new ElapsedTime();

    private double lastPeriodicTime = 0;
    private double maxPeriodicTime = 0;
    private double avgPeriodicTime = 0;
    private int callCount = 0;

    public TimedSubsystem(Subsystem wrapped) {
        this.wrapped = wrapped;
    }

    @Override
    public void periodic() {
        timer.reset();
        wrapped.periodic();

        lastPeriodicTime = timer.milliseconds();
        maxPeriodicTime = Math.max(maxPeriodicTime, lastPeriodicTime);

        callCount++;
        avgPeriodicTime = ((avgPeriodicTime * (callCount - 1)) + lastPeriodicTime / callCount);
    }

    @Override
    public void init() {
        wrapped.init();
    }

    public void stop() {
        wrapped.stop();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry, TelemetryPacket packet, Logger logger) {
        wrapped.updateTelemetry(telemetry, packet, logger);

        packet.put(getName() + " Last", lastPeriodicTime);
        packet.put(getName() + " Avg", avgPeriodicTime);
        packet.put(getName() + " Max", maxPeriodicTime);

        logger.put(getName() + " Last", lastPeriodicTime);
        logger.put(getName() + " Avg", avgPeriodicTime);
        logger.put(getName() + " Max", maxPeriodicTime);
    }

    @Override
    public String getName() {
        return wrapped.getName() + " [Timed]";
    }

    @Override
    public boolean isHealthy() {
        return wrapped.isHealthy();
    }

    public double getLastPeriodicTime () {
        return lastPeriodicTime;
    }
}
