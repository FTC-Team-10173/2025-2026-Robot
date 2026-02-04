package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Logger;

public class Shooter extends SubsystemBase {
    private final MotorGroup flywheel;
    private double targetPower = 0;
    private double targetVelocity = 0;
    private boolean isRunning = false;

    public Shooter(HardwareMap hardwareMap) {
        flywheel = new MotorGroup(
                new Motor(hardwareMap, "flywheel_left", Motor.GoBILDA.BARE),
                new Motor(hardwareMap, "flywheel_right", Motor.GoBILDA.BARE)
        );

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
        telemetry.addData(getName() + " Velocity", getVelocity());
        telemetry.addData(getName() + " Target", getTargetVelocity());
        telemetry.addData(getName() + " Ready", isReady());

        if (logger != null) {
            logger.put(getName() + " Healthy", isHealthy());
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