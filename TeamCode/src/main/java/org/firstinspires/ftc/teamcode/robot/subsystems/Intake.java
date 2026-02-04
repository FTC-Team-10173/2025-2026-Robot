package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Logger;

public class Intake extends SubsystemBase {
    private final Motor intakeMotor;
    private final ServoEx feedGate;
    private final double openAngle;
    private final double closedAngle;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_435);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setInverted(false);

        openAngle = Constants.Gate.OPEN_ANGLE;
        closedAngle = Constants.Gate.CLOSED_ANGLE;

        feedGate = new SimpleServo(
                hardwareMap, "feedGate",
                Constants.Gate.MIN_ANGLE,
                Constants.Gate.MAX_ANGLE,
                AngleUnit.DEGREES
        );
        feedGate.setInverted(false);
    }

    public void setPower(double power) {
        intakeMotor.set(power);
    }

    public void setGateAngle(double angle) {
        feedGate.turnToAngle(angle);
    }

    public void openGate() {
        setGateAngle(openAngle);
    }

    public void closeGate() {
        setGateAngle(closedAngle);
    }

    public void halfIntake() {
        closeGate();
        setPower(1.0);
    }

    public void fullIntake() {
        openGate();
        setPower(1.0);
    }

    public void outtake() {
        setPower(-1.0);
        openGate();
    }

    public void stopIntake() {
        setPower(0);
        closeGate();
    }

    public void stop() {
        stopIntake();
    }

    public boolean isHealthy() {
        return intakeMotor != null && feedGate != null;
    }

    public void updateTelemetry(Telemetry telemetry, Logger logger) {
        telemetry.addData(getName() + " Healthy", isHealthy());

        if (logger != null) {
            logger.put(getName() + " Healthy", isHealthy());
        }
    }

    public Action intake(double power) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                closeGate();
                setPower(power);

                return false;
            }
        };
    }

    public Action intake(double power, double time) {
        return new Action() {
            private double startTime = -1;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double t;
                if (startTime < 0) {
                    startTime = Actions.now();
                    t = 0;
                    closeGate();
                    setPower(power);
                } else {
                    t = Actions.now() - startTime;
                }

                // stop after time has elapsed
                if (t >= time) {
                    stopIntake();
                    return false;
                }
                return true;
            }
        };
    }

    public Action feed(double power, double time) {
        return new Action() {
            private double startTime = -1;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double t;
                if (startTime < 0) {
                    startTime = Actions.now();
                    t = 0;
                    openGate();
                    setPower(power);
                } else {
                    t = Actions.now() - startTime;
                }

                // stop after time has elapsed
                if (t >= time) {
                    stopIntake();
                    closeGate();
                    return false;
                }
                return true;
            }
        };
    }

    public Action open() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                openGate();

                return false;
            }
        };
    }
}