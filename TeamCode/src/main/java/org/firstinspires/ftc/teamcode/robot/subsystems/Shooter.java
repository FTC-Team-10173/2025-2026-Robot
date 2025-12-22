package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.robot.DriverControls;

public class Shooter {
    public MotorGroup flywheel;
    DriverControls controls;
    VisionController vision;
    public Double power = 0.5;
    double targetVel = 1200;
    LED led;

    // PIDF coefficients
    public static class Params {
        public static final double kP = 20;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 0.7;
        public static final double kA = 0;
    }

    // TeleOp constructor
    public Shooter(HardwareMap hardwareMap, DriverControls controls, LED ledSubsystem, VisionController vision) {
        // initialize motors as a motor group
        flywheel = new MotorGroup(
                new Motor(hardwareMap, "flywheel_left", Motor.GoBILDA.BARE),
                new Motor(hardwareMap, "flywheel_right", Motor.GoBILDA.BARE)
        );

        // configure motor settings
        flywheel.setInverted(true);
        flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        flywheel.setRunMode(Motor.RunMode.VelocityControl);

        // set PIDF and feedforward coefficients
        flywheel.setVeloCoefficients(Params.kP, Params.kI, Params.kD);
        flywheel.setFeedforwardCoefficients(Params.kS, Params.kV, Params.kA);

        // store driver controls
        this.controls = controls;

        // store vision controller
        this.vision = vision;

        // LED subsystem
        led = ledSubsystem;
    }

    // Autonomous constructor
    public Shooter(HardwareMap hardwareMap, LED ledSubsystem, VisionController vision) {
        // initialize motors as a motor group
        flywheel = new MotorGroup(
                new Motor(hardwareMap, "flywheel_left", Motor.GoBILDA.BARE),
                new Motor(hardwareMap, "flywheel_right", Motor.GoBILDA.BARE)
        );

        // configure motor settings
        flywheel.setInverted(true);
        flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        flywheel.setRunMode(Motor.RunMode.VelocityControl);

        // set PIDF and feedforward coefficients
        flywheel.setVeloCoefficients(Params.kP, Params.kI, Params.kD);
        flywheel.setFeedforwardCoefficients(Params.kS, Params.kV, Params.kA);

        // store vision controller
        this.vision = vision;

        // LED subsystem
        led = ledSubsystem;
    }

    public void periodic() {
        // adjust power based on tag distance
        setPower(vision.distance);

        // set flywheel spin based on driver controls
        spin(controls.spinShooterPressed());
    }

    // set flywheel power and update LED status
    public void spin(boolean bumper) {
        if (bumper) { // spin flywheel
            // set flywheel power
            flywheel.set(power);

            // get current velocity
            double vel = flywheel.getVelocity();

            // update LED status
            led.spinningUp = true;
            led.shooterReady = vel >= (targetVel - 20);
        } else {
            // stop flywheel
            flywheel.set(0);

            // update LED status
            led.shooterReady = false;
            led.spinningUp = false;
        }
    }

    // adjust power based on tag distance
    public void setPower(double tag_distance) {
        if (tag_distance != -1) {
            // adjust power using a scaling formula
            double scale = 670;
            double intercept = 85;

            // power formula derived from testing
            power = Math.sqrt(tag_distance + intercept) / Math.sqrt(scale + intercept);

            // update target velocity
            targetVel = 2400 * power;
        }
    }

    // maintain flywheel velocity and update LED status
    public Action maintainVelocity() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // set flywheel power
                flywheel.set(power);

                // get current velocity
                double vel = flywheel.getVelocity();
                packet.put("shooterVelocity", vel);

                // update LED status
                if (targetVel == 0) {
                    led.shooterReady = false;
                    led.spinningUp = false;
                } else {
                    led.spinningUp = true;
                    led.shooterReady = vel >= (targetVel - 20);
                }

                // run indefinitely
                return true;
            }
        };
    }

    // spin up flywheel to target velocity
    public Action spinUp(double newPower, double speedPercent) {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // set new power
                power = newPower;

                // get current velocity and target velocity
                double vel = flywheel.getVelocity();
                targetVel = 2400 * newPower;

                // return true if still spinning up
                return ((targetVel - 20) * speedPercent) > vel;
            }
        };
    }
}
