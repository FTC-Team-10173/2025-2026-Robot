package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Logger;

public class Intake extends SubsystemBase {
    private final Motor intakeMotor;

    public Intake(HardwareMap hardwareMap) {
        // configure motor
        intakeMotor = new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_312);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setInverted(false);
    }

    public void setPower(double power) {
        intakeMotor.set(power);
    }

    public void intake() {
        setPower(1.0);
    }

    public void farFeed() {
        setPower(0.5);
    }

    public void outtake() {
        setPower(-1.0);
    }

    public void stopIntake() {
        setPower(0);
    }

    public void stop() {
        stopIntake();
    }

    public boolean isHealthy() {
        return intakeMotor != null;
    }

    public void updateTelemetry(Telemetry telemetry, Logger logger) {
        telemetry.addData(getName() + " Healthy", isHealthy());

        if (logger != null) {
            logger.put(getName() + " Healthy", isHealthy());
        }
    }

    // RoadRunner Action for setting intake power
    public Action intake(double power) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setPower(power);

                return false;
            }
        };
    }

    // RoadRunner Action for setting intake power for specified time
    public Action intake(double power, double time) {
        return new Action() {
            private double startTime = -1;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double t;
                if (startTime < 0) {
                    startTime = Actions.now();
                    t = 0;
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

    // RoadRunner Action for feeding artifacts for specified time
    public Action feed(double power, double time) {
        return new Action() {
            private double startTime = -1;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double t;
                if (startTime < 0) {
                    startTime = Actions.now();
                    t = 0;
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
}