package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.DriverControls;

public class Intake {

    Motor intakeMotor;
    CRServo feederServo;
    DriverControls controls;
    LED led;

    // TeleOp constructor
    public Intake(HardwareMap hardwareMap, DriverControls controls, LED ledSubsystem) {
        // initialize motors and servos
        intakeMotor = new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_435);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        feederServo = new CRServo(hardwareMap, "feeder");

        // store driver controls
        this.controls = controls;

        // LED subsystem
        led = ledSubsystem;
    }

    // Autonomous constructor
    public Intake(HardwareMap hardwareMap, LED ledSubsystem) {
        // initialize motors and servos
        intakeMotor = new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_435);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        feederServo = new CRServo(hardwareMap, "feeder");

        // LED subsystem
        led = ledSubsystem;
    }

    // periodic method to be called in main loop
    public void periodic() {
        // set intake and feeder power based on driver controls
        setPower(
                controls.intakePower(),
                controls.fullIntakePressed(),
                controls.topOuttakePressed()
        );
    }

    // set intake and feeder power
    public void setPower(double half, boolean full, boolean top) {
        if (full) { // full intake
            intakeMotor.set(1);
            feederServo.set(-1);
        } else if (half > 0.1) { // bottom half intake (motor only)
            intakeMotor.set(half);
        } else if (half < -0.1) { // full outtake
            intakeMotor.set(half);
            feederServo.set(-half);
        } else if (top) { // top feeder only in reverse
            feederServo.set(1);
        } else { // stop
            intakeMotor.set(0);
            feederServo.set(0);
        }
    }

    // feed for a certain time
    public Action feed(double power, double time) {
        return new Action() {
            // timer to track elapsed time
            final ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // set intake and feeder power
                intakeMotor.set(power);
                feederServo.set(-power);

                // stop after time has elapsed
                if (timer.milliseconds() >= time) {
                    intakeMotor.set(0);
                    feederServo.set(0);
                    return false;
                } else {
                    return true;
                }
            }
        };
    }

    // feed after a delay
    public Action feedDelay(double power, double delay) {
        return new Action() {
            // timer to track elapsed time
            final ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // start feeding after delay
                if (timer.milliseconds() >= delay) {
                    // set intake and feeder power
                    intakeMotor.set(power);
                    feederServo.set(-power);
                    return false;
                } else {
                    return true;
                }
            }
        };
    }

    // feed indefinitely
    public Action feed(double power) {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // set intake and feeder power
                intakeMotor.set(power);
                feederServo.set(-power);

                return false;
            }
        };
    }

    // intake for a certain time
    public Action intake(double power, double delay) {
        return new Action() {
            // timer to track elapsed time
            final ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // set intake and feeder power
                intakeMotor.set(power);
                feederServo.set(-power);

                // stop after delay has elapsed
                return timer.milliseconds() < delay;
            }
        };
    }

    // intake indefinitely
    public Action intake(double power) {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // set intake and feeder power
                intakeMotor.set(power);
                feederServo.set(-power);

                return false;
            }
        };
    }
}
