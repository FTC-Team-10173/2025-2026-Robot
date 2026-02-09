package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Logger;

public class Turret extends SubsystemBase {
    private final ServoEx turret0;
    private final ServoEx turret1;
    private double targetAngle = Constants.Turret.MAX_ANGLE / 2;
    public Turret(HardwareMap hardwareMap) {
        turret0 = new SimpleServo(
                hardwareMap, "turret0",
                Constants.Turret.RANGE_MIN_ANGLE,
                Constants.Turret.RANGE_MAX_ANGLE,
                AngleUnit.DEGREES
        );
        turret1 = new SimpleServo(
                hardwareMap, "turret1",
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

    public void set(double targetAngle) {
        targetAngle = Math.min(
                Constants.Turret.MAX_ANGLE,
                Math.max(
                        Constants.Turret.MIN_ANGLE,
                        targetAngle
                )
        );
        this.targetAngle = targetAngle;
    }

    public boolean isHealthy() {
        return turret0 != null && turret1 != null;
    }

    public void updateTelemetry(Telemetry telemetry, Logger logger) {
        telemetry.addData(getName() + " Healthy", isHealthy());

        if (logger != null) {
            logger.put(getName() + " Healthy", isHealthy());
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

    public static class TurretPosition {
        public double turret0;
        public double turret1;

        public TurretPosition(double turret0, double turret1) {
            this.turret0 = turret0;
            this.turret1 = turret1;
        }
    }
}
