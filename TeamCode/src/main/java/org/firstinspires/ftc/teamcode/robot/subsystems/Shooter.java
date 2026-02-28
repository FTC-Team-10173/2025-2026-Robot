package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Logger;

@Config
public class Shooter extends SubsystemBase {
    private final MotorGroup flywheel;
    private double targetPower = 0;
    private double targetVelocity = 0;
    private boolean isRunning = false;
    Motor leftFlywheel;
    Motor rightFlywheel;

    double testPower = 0.1;

    public Shooter(HardwareMap hardwareMap) {
        leftFlywheel = getMotor(hardwareMap, "leftFlywheel", false, Motor.GoBILDA.BARE);
        rightFlywheel = getMotor(hardwareMap, "rightFlywheel", true, Motor.GoBILDA.BARE);

        flywheel = new MotorGroup(leftFlywheel, rightFlywheel);

        flywheel.setInverted(false);
        flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        flywheel.setRunMode(Motor.RunMode.VelocityControl);

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
    }

    @NonNull
    public Motor getMotor(HardwareMap hardwareMap, String id, Boolean inverted, Motor.GoBILDA type) {
        Motor motor = new Motor(hardwareMap, id, type);
        motor.setInverted(inverted);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor.setRunMode(Motor.RunMode.VelocityControl);

        motor.setVeloCoefficients(
                Constants.Shooter.kP,
                Constants.Shooter.kI,
                Constants.Shooter.kD
        );
        motor.setFeedforwardCoefficients(
                Constants.Shooter.kS,
                Constants.Shooter.kV,
                Constants.Shooter.kA
        );

        return motor;
    }

    @Override
    public void periodic() {
        if (isRunning) {
            flywheel.set(targetPower);
        } else {
            flywheel.set(0);
        }
    }

    public void setPower(double power) {
        this.targetPower = power;
        this.targetVelocity = Constants.Shooter.MAX_RPM * power;
    }

    public void startFlywheel() {
        isRunning = true;
    }

    public void stopFlywheel() {
        isRunning = false;
    }

    public void stop() {
        isRunning = false;
    }

    public void test(boolean positive) {
        double additive = positive ? 0.005 : -0.005;

        testPower += additive;
    }

    public double getTest() {
        return testPower;
    }

    public boolean isReady() {
        return getVelocity() >= (targetVelocity - Constants.Shooter.VELOCITY_TOLERANCE);
    }

    public boolean isReady(double speedPercent) {
        return getVelocity() >= (targetVelocity * speedPercent - Constants.Shooter.VELOCITY_TOLERANCE);
    }

    public double getVelocity() {
        return flywheel.getVelocity();
    }

    public double getTargetPower() {
        return targetPower;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public boolean isRunning() {
        return isRunning;
    }

    public boolean isHealthy() {
        return flywheel != null;
    }

    public void updateTelemetry(Telemetry telemetry, Logger logger) {
        telemetry.addData(getName() + " Healthy", isHealthy());
        telemetry.addData(getName() + " Test", getTest());
        telemetry.addData(getName() + " Power", getTargetPower());
        telemetry.addData(getName() + " Velocity", getVelocity());
        telemetry.addData(getName() + " Target", getTargetVelocity());
        telemetry.addData(getName() + " Ready", isReady());

        if (logger != null) {
            logger.put(getName() + " Healthy", isHealthy());
            logger.put(getName() + " Power", getTargetPower());
            logger.put(getName() + " Velocity", getVelocity());
            logger.put(getName() + " Target", getTargetVelocity());
            logger.put(getName() + " Ready", isReady());
        }
    }

    public Action maintainVelocity() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (isRunning) {
                    flywheel.set(targetPower);
                } else {
                    flywheel.set(0);
                }

                return true;
            }
        };
    }

    public Action startShooter() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                isRunning = true;

                return !isReady();
            }
        };
    }

    public Action stopShooter() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                isRunning = false;

                return false;
            }
        };
    }
}