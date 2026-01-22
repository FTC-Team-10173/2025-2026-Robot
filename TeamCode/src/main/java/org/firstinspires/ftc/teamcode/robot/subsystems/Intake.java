package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.DriverControls;
import org.firstinspires.ftc.teamcode.robot.Logger;

public class Intake implements Subsystem {

    Motor intakeMotor;
    ServoEx feedGate;
    DriverControls controls;
    double OPEN_ANGLE;
    double CLOSED_ANGLE;

    private final ElapsedTime timer = new ElapsedTime();
    private boolean timed = false;

    // TeleOp constructor
    public Intake(HardwareMap hardwareMap, DriverControls controls) {
        // initialize motor
        intakeMotor = new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_435);

        // configure motor settings
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setInverted(true);
        
        OPEN_ANGLE = Constants.Gate.OPEN_ANGLE;
        CLOSED_ANGLE = Constants.Gate.CLOSED_ANGLE;

        // initialize gate servo
        feedGate = new SimpleServo(
                hardwareMap, "feedGate", Constants.Gate.MIN_ANGLE, Constants.Gate.MAX_ANGLE, AngleUnit.DEGREES
        );

        // configure gate servo
        feedGate.setInverted(false);

        // store driver controls
        this.controls = controls;
    }

    // Autonomous constructor
    public Intake(HardwareMap hardwareMap) {
        // initialize motor
        intakeMotor = new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_435);

        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setInverted(true);

        OPEN_ANGLE = Constants.Gate.OPEN_ANGLE;
        CLOSED_ANGLE = Constants.Gate.CLOSED_ANGLE;

        // initialize gate servo
        feedGate = new SimpleServo(
                hardwareMap, "feedGate", Constants.Gate.MIN_ANGLE, Constants.Gate.MAX_ANGLE, AngleUnit.DEGREES
        );

        // configure gate servo
        feedGate.setInverted(false);
    }

    // periodic method to be called in main loop
    public void periodic() {
        // set intake and feeder power based on driver controls
        setPower(
                controls.intakePower(),
                controls.fullIntakePressed()
        );
    }

    public double getGateAngle() {
        return feedGate.getAngle(AngleUnit.DEGREES);
    }

    public boolean isHealthy() {
        return intakeMotor != null && feedGate != null;
    }

    public void stop() {
        intakeMotor.set(0);
    }

    public void updateTelemetry(Telemetry telemetry, TelemetryPacket packet, Logger logger) {
        telemetry.addLine();
        telemetry.addData(getName() + " Intake Power", "%.2f", intakeMotor.get());
        telemetry.addData(getName() + " Gate Angle", "%.2f", feedGate.getAngle(AngleUnit.DEGREES));
        telemetry.addData(getName() + " Healthy", isHealthy());

        logger.put(getName() + " Intake Power", intakeMotor.get());
        logger.put(getName() + " Gate Angle", feedGate.getAngle(AngleUnit.DEGREES));
    }

    // set intake and feeder power
    public void setPower(double half, boolean full) {
        if (full) { // full intake
            if (!timed) {timed = true; timer.reset();}
            feedGate.turnToAngle(OPEN_ANGLE);
            if (timer.milliseconds() >= 800) {
                intakeMotor.set(1);
            }
        } else if (half > 0.1) { // bottom half intake (motor only)
            timed = false;
            intakeMotor.set(half);
            feedGate.turnToAngle(CLOSED_ANGLE);
        } else if (half < -0.1) { // full outtake
            timed = false;
            intakeMotor.set(half);
            feedGate.turnToAngle(CLOSED_ANGLE);
        } else { // stop
            timed = false;
            intakeMotor.set(0);
            feedGate.turnToAngle(CLOSED_ANGLE);
        }
    }

    // feed for a certain time
    public Action feed(double power, double time) {
        return new Action() {
            private double startTime = -1;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double t;
                if (startTime < 0) {
                    startTime = Actions.now();
                    t = 0;
                } else {
                    t = Actions.now() - startTime;
                }

                // set intake and feeder power
                intakeMotor.set(power);
                feedGate.turnToAngle(OPEN_ANGLE);

                // stop after time has elapsed
                if (t >= time) {
                    intakeMotor.set(0);
                    feedGate.turnToAngle(CLOSED_ANGLE);
                    startTime = -1;
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
            private double startTime = -1;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double t;
                if (startTime < 0) {
                    startTime = Actions.now();
                    t = 0;
                } else {
                    t = Actions.now() - startTime;
                }

                // start feeding after delay
                if (t >= delay) {
                    // set intake and feeder power
                    intakeMotor.set(power);
                    feedGate.turnToAngle(OPEN_ANGLE);
                    startTime = -1;
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

    public Action open() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                feedGate.turnToAngle(OPEN_ANGLE);

                return false;
            }
        };
    }

    public Action close() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                feedGate.turnToAngle(CLOSED_ANGLE);

                return false;
            }
        };
    }

    // intake for a certain time
    public Action intake(double power, double delay) {
        return new Action() {
            private double startTime = -1;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double t;
                if (startTime < 0) {
                    startTime = Actions.now();
                    t = 0;
                } else {
                    t = Actions.now() - startTime;
                }

                // set intake and feeder power
                intakeMotor.set(power);
                feedGate.turnToAngle(CLOSED_ANGLE);

                if (t >= delay) {
                    startTime = -1;
                    return false;
                } else {
                    return true;
                }
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
