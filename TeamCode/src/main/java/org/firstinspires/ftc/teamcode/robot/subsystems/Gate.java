package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Logger;

public class Gate extends SubsystemBase {
    private final double openAngle;
    private final double closedAngle;
    private final ServoEx leftGate;
    private final ServoEx rightGate;

    public Gate(HardwareMap hardwareMap) {
        openAngle = Constants.Gate.OPEN_ANGLE;
        closedAngle = Constants.Gate.CLOSED_ANGLE;

        // configure gate servo
        leftGate = new SimpleServo(
                hardwareMap, "leftGate",
                Constants.Gate.MIN_ANGLE,
                Constants.Gate.MAX_ANGLE,
                AngleUnit.DEGREES
        );
        leftGate.setInverted(true);

        rightGate = new SimpleServo(
                hardwareMap, "rightGate",
                Constants.Gate.MIN_ANGLE,
                Constants.Gate.MAX_ANGLE,
                AngleUnit.DEGREES
        );
        rightGate.setInverted(false);
    }

    private void set(double angle) {
        leftGate.turnToAngle(angle);
        rightGate.turnToAngle(angle);
    }

    public boolean isHealthy() {
        return leftGate != null && rightGate != null;
    }

    public void updateTelemetry(Telemetry telemetry, Logger logger) {
        telemetry.addData(getName() + " Healthy", isHealthy());

        if (logger != null) {
            logger.put(getName() + " Healthy", isHealthy());
        }
    }

    public void open() {
        set(openAngle);
    }

    public void close() {
        set(closedAngle);
    }

    public Action openAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                open();

                return false;
            }
        };
    }

    public Action closeAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                close();

                return false;
            }
        };
    }
}
