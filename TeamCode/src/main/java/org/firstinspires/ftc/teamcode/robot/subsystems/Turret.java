package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Logger;
import org.firstinspires.ftc.teamcode.robot.ShooterMath;

public class Turret extends SubsystemBase {
    private final ServoEx turret0;
    private final ServoEx turret1;
    private double targetAngle = Constants.Turret.RANGE_MAX_ANGLE / 2;
    private boolean locked = false;
    private double turretDegrees = 0;

    public static final double CENTER = Constants.Turret.RANGE_MAX_ANGLE / 2.0;

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

    public void set(double turretTarget) {
        if (!locked) {
            turretDegrees = turretTarget;

            double servoDegrees = turretTarget * Constants.Turret.GEAR_RATIO;

            double servoTarget = (Constants.Turret.RANGE_MAX_ANGLE / 2.0) + servoDegrees;

            double center = Constants.Turret.RANGE_MAX_ANGLE / 2.0;

            double maxServoOffset = Constants.Turret.RANGE * Constants.Turret.GEAR_RATIO;

            double min = center - maxServoOffset;
            double max = center + maxServoOffset;

            servoTarget = Math.max(min, Math.min(max, servoTarget));

            this.targetAngle = servoTarget;
        } else {
            turretDegrees = 0;
        }
    }

    public void autoSet(double turretTarget) {
        turretDegrees = turretTarget;

        double servoDegrees = turretTarget * Constants.Turret.GEAR_RATIO;

        double servoTarget = (Constants.Turret.RANGE_MAX_ANGLE / 2.0) + servoDegrees;

        double maxServoOffset = Constants.Turret.RANGE * Constants.Turret.GEAR_RATIO;

        double min = CENTER - maxServoOffset;
        double max = CENTER + maxServoOffset;

        servoTarget = Math.max(min, Math.min(max, servoTarget));

        turret0.turnToAngle(servoTarget);
        turret1.turnToAngle(servoTarget);
    }

    public boolean isHealthy() {
        return turret0 != null && turret1 != null;
    }

    public void updateTelemetry(Telemetry telemetry, Logger logger) {
        telemetry.addData(getName() + " Healthy", isHealthy());
        telemetry.addData(getName() + " targetAngle", targetAngle);
        telemetry.addData(getName() + " turretDegrees", turretDegrees);

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

    public double getTurretDegrees() {
        return turretDegrees;
    }

    public void lockTurret() {
        if (!locked) {
            locked = true;
            targetAngle = Constants.Turret.RANGE_MAX_ANGLE / 2;
        } else {
            locked = false;
        }
    }

    public Action goTo(Pose2d targetPose, Constants.Alliance alliance) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                targetAngle = ShooterMath.getTurretTarget(targetPose, alliance);

                return false;
            }
        };
    }

    public Action maintainHeading() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                autoSet(targetAngle);

                return true;
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
