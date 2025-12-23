package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.DriverControls;
import org.firstinspires.ftc.teamcode.robot.RobotState;

public class Shooter implements Subsystem {
    public MotorGroup flywheel;
    DriverControls controls;
    RobotState robotState;
    Vision vision;
    public Double power = 0.5;
    public double targetVel = 1200;

    // TeleOp constructor
    public Shooter(
            HardwareMap hardwareMap,
            DriverControls controls,
            RobotState robotState,
            Vision vision
    ) {
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
        flywheel.setVeloCoefficients(
                Constants.Shooter.kP,
                Constants.Shooter.kI,
                Constants.Shooter.kD
        );
        flywheel.setFeedforwardCoefficients(
                Constants.Shooter.kS,
                Constants.Shooter.kV,
                Constants.Shooter.kA
        );

        // store driver controls
        this.controls = controls;

        // store robot state
        this.robotState = robotState;

        // store vision controller
        this.vision = vision;
    }

    // Autonomous constructor
    public Shooter(
            HardwareMap hardwareMap,
            RobotState robotState,
            Vision vision
    ) {
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
        flywheel.setVeloCoefficients(
                Constants.Shooter.kP,
                Constants.Shooter.kI,
                Constants.Shooter.kD
        );
        flywheel.setFeedforwardCoefficients(
                Constants.Shooter.kS,
                Constants.Shooter.kV,
                Constants.Shooter.kA
        );

        // store robot state
        this.robotState = robotState;

        // store vision controller
        this.vision = vision;
    }

    // periodic method to be called in main loop
    public void periodic() {
        // adjust power based on tag distance
        setPower(vision.distance);

        // set flywheel spin based on driver controls
        spin(controls.spinShooterPressed());
    }

    // stop the subsystem safely
    @Override
    public void stop() {
        flywheel.set(0);
        robotState.shooterReady = false;
    }

    // add telemetry data for this subsystem
    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData(getName() + " Power", "%.2f", power);
        telemetry.addData(getName() + " Target", "%.0f RPM", targetVel);
        telemetry.addData(getName() + " Current", "%.0f RPM", flywheel.getVelocity());
        telemetry.addData(getName() + " Ready", robotState.shooterReady);
        telemetry.addData(getName() + " Healthy", isHealthy());
    }

    // check if shooter is healthy (motors responding correctly)
    @Override
    public boolean isHealthy() {
        // Check if motors are connected and responding
        // Could add more sophisticated health checks here
        return flywheel != null;
    }

    // set flywheel power and update LED status
    public void spin(boolean bumper) {
        if (bumper) { // spin flywheel
            // set flywheel power
            flywheel.set(power);

            // update shooter ready status
            robotState.shooterReady = isReady();
        } else {
            // stop flywheel
            flywheel.set(0);

            // update shooter ready status
            robotState.shooterReady = false;
        }
    }

    // adjust power based on tag distance
    public void setPower(double tag_distance) {
        if (tag_distance != -1) {

            // power formula derived from testing
            power = Math.sqrt(tag_distance + Constants.Shooter.INTERCEPT)
                    / Math.sqrt(Constants.Shooter.SCALE + Constants.Shooter.INTERCEPT);

            // update target velocity
            targetVel = Constants.Shooter.MAX_RPM * power;
        }
    }

    // check if shooter is at target speed and ready to fire
    public boolean isReady() {
        return getVelocity() >= (targetVel - Constants.Shooter.VELOCITY_TOLERANCE);
    }

    public boolean isReady(double speedPercent) {
        return getVelocity() >= ((targetVel - Constants.Shooter.VELOCITY_TOLERANCE) * speedPercent);
    }

    // get current velocity
    public double getVelocity() {
        return flywheel.getVelocity();
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

                // update shooter ready status
                robotState.shooterReady = targetVel != 0 && isReady();

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
                targetVel = Constants.Shooter.MAX_RPM * newPower;

                // return true if still spinning up
                return !isReady(speedPercent);
            }
        };
    }
}
