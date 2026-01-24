package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Logger;

public class Intake extends SubsystemBase {
    private final Motor intakeMotor;
    private final ServoEx feedGate;
    private final ElapsedTime timer = new ElapsedTime();
    private final double openAngle;
    private final double closedAngle;
    private boolean timed = false;

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
        timed = false;
        setPower(1.0);
        closeGate();
    }

    public void fullIntake() {
        if (!timed) {
            timed = true;
            timer.reset();
        }

        openGate();
        if (timer.milliseconds() >= 800) {
            setPower(1.0);
        }
    }

    public void outtake() {
        timed = false;
        setPower(-1.0);
        openGate();
    }

    public void stopIntake() {
        timed = false;
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

    public class intake implements Action {
        public intake(double power) {
            closeGate();
            setPower(power);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return false;
        }
    }

    public Action intake(double power) {
        return new intake(power);
    }

    public class intakeTime implements Action {
        private final double time;
        private double startTime;
        public intakeTime(double power, double time) {
            closeGate();
            setPower(power);
            this.time = time;
            this.startTime = -1;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double t;
            if (startTime < 0) {
                startTime = System.nanoTime();
                t = 0;
            } else {
                t = System.nanoTime() - startTime;
            }

            // stop after time has elapsed
            if (t >= time) {
                stopIntake();
                return false;
            }
            return true;
        }
    }

    public Action intake(double power, double time) {
        return new intakeTime(power, time);
    }

    public class feed implements Action {
        private final double time;
        private double startTime;
        public feed(double power, double time) {
            openGate();
            setPower(power);
            this.time = time;
            this.startTime = -1;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double t;
            if (startTime < 0) {
                startTime = System.nanoTime();
                t = 0;
            } else {
                t = System.nanoTime() - startTime;
            }

            // stop after time has elapsed
            if (t >= time) {
                stopIntake();
                return false;
            }
            return true;
        }
    }

    public Action feed(double power, double time) {
        return new feed(power, time);
    }

    public class open implements Action {
        public open() {
            openGate();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return false;
        }
    }

    public Action open() {
        return new open();
    }
}