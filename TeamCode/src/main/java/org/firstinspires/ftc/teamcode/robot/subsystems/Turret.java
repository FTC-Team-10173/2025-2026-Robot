package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Logger;
import org.firstinspires.ftc.teamcode.robot.ShooterMath;

public class Turret extends SubsystemBase {
    private final ServoEx turret0;
    private final ServoEx turret1;
    private double targetAngle = Constants.Turret.RANGE_MAX_ANGLE / 2;
    public Turret(HardwareMap hardwareMap) {
        turret0 = new SimpleServo(
                hardwareMap, "leftTurret",
                Constants.Turret.RANGE_MIN_ANGLE,
                Constants.Turret.RANGE_MAX_ANGLE,
                AngleUnit.DEGREES
        );
        turret1 = new SimpleServo(
                hardwareMap, "rightTurret",
                Constants.Turret.RANGE_MIN_ANGLE,
                Constants.Turret.RANGE_MAX_ANGLE,
                AngleUnit.DEGREES
        );

        turret0.setInverted(false);
        turret1.setInverted(false);
    }

    @Override
    public void periodic() {
        turret0.turnToAngle(targetAngle);
        turret1.turnToAngle(targetAngle);
    }

    public void set(double servoTarget) {
        double center = Constants.Turret.RANGE_MAX_ANGLE / 2.0;

        double maxServoOffset = Constants.Turret.RANGE / Constants.Turret.GEAR_RATIO;

        double min = center - maxServoOffset;
        double max = center + maxServoOffset;

        servoTarget = Math.max(min, Math.min(max, servoTarget));

        this.targetAngle = servoTarget;
    }

    public boolean isHealthy() {
        return turret0 != null && turret1 != null;
    }

    public void updateTelemetry(Telemetry telemetry, Logger logger) {
        telemetry.addData(getName() + " Healthy", isHealthy());
        telemetry.addData(getName() + " targetAngle", targetAngle);

        if (logger != null) {
            logger.put(getName() + " Healthy", isHealthy());
            logger.put(getName() + " targetAngle", targetAngle);
        }
    }

    public void stop() {

    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public TurretPosition getPosition() {
        return new TurretPosition(
                turret0.getPosition(),
                turret1.getPosition()
        );
    }

    public Action goTo(Pose2d targetPose, Constants.Alliance alliance) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double targetHeading = ShooterMath.getTurretTarget(targetPose, alliance);

                set(targetHeading);

                return false;
            }
        };
    }

    public static class TurretPosition {
        public double turret0;
        public double turret1;

        public TurretPosition(double turret0, double turret1) {
            this.turret0 = turret0;
            this.turret1 = turret1;
        }
    }
}
