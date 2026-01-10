package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.DriverControls;

public class Intake implements Subsystem {

    Motor intakeMotor;
    ServoEx feedGate;
    DriverControls controls;
    
    double OPEN_ANGLE;
    double CLOSED_ANGLE;

    // TeleOp constructor
    public Intake(HardwareMap hardwareMap, DriverControls controls) {
        // initialize motor
        intakeMotor = new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_435);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        
        OPEN_ANGLE = Constants.Gate.OPEN_ANGLE;
        CLOSED_ANGLE = Constants.Gate.CLOSED_ANGLE;

        // initialize gate servo
        feedGate = new SimpleServo(
                hardwareMap, "feedGate", OPEN_ANGLE, CLOSED_ANGLE, AngleUnit.DEGREES
        );

        // configure gate servo
        feedGate.setInverted(false);
        feedGate.setPosition(0);

        // store driver controls
        this.controls = controls;
    }

    // Autonomous constructor
    public Intake(HardwareMap hardwareMap) {
        // initialize motor
        intakeMotor = new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_435);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // initialize gate servo
        feedGate = new SimpleServo(
                hardwareMap, "servo_name", OPEN_ANGLE, CLOSED_ANGLE, AngleUnit.DEGREES
        );

        // configure gate servo
        feedGate.setInverted(false);
        feedGate.setPosition(0);
    }

    // periodic method to be called in main loop
    public void periodic() {
        // set intake and feeder power based on driver controls
        setPower(
                controls.intakePower(),
                controls.fullIntakePressed()
        );
    }

    public boolean isHealthy() {
        return intakeMotor != null && feedGate != null;
    }

    public void stop() {
        intakeMotor.set(0);
        feedGate.turnToAngle(OPEN_ANGLE);
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine();
        telemetry.addData(getName() + " Intake Power", "%.2f", intakeMotor.get());
        telemetry.addData(getName() + " Gate Angle", "%.2f", feedGate.getAngle(AngleUnit.DEGREES));
        telemetry.addData(getName() + " Healthy", isHealthy());
    }

    // set intake and feeder power
    public void setPower(double half, boolean full) {
        if (full) { // full intake
            intakeMotor.set(1);
            feedGate.turnToAngle(OPEN_ANGLE);
        } else if (half > 0.1) { // bottom half intake (motor only)
            intakeMotor.set(half);
            feedGate.turnToAngle(CLOSED_ANGLE);
        } else if (half < -0.1) { // full outtake
            intakeMotor.set(half);
            feedGate.turnToAngle(CLOSED_ANGLE);
        } else { // stop
            intakeMotor.set(0);
            feedGate.turnToAngle(CLOSED_ANGLE);
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
                feedGate.turnToAngle(OPEN_ANGLE);

                // stop after time has elapsed
                if (timer.milliseconds() >= time) {
                    intakeMotor.set(0);
                    feedGate.turnToAngle(CLOSED_ANGLE);
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
                    feedGate.turnToAngle(OPEN_ANGLE);
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
                feedGate.turnToAngle(OPEN_ANGLE);

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
                feedGate.turnToAngle(CLOSED_ANGLE);

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
                feedGate.turnToAngle(CLOSED_ANGLE);

                return false;
            }
        };
    }
}
