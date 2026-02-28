package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.roadrunner.*;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Logger;

import java.lang.Math;

public class Drive extends SubsystemBase {
    public final MecanumDrive drive;
    private final IMU imu;
    private final PIDController headingPID;
    private double leftY = 0;
    private double leftX = 0;
    private double rightX = 0;
    private boolean useHeadingLock = false;
    private double headingLockError = 0;
    private double turnPower;

    public Drive(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");

        Pose2d startPose = new Pose2d(0, 0, 0);
        if (Constants.BlackBoard.containsKey(Constants.Keys.POSE)) {
            startPose = (Pose2d) Constants.BlackBoard.get(Constants.Keys.POSE);
        }

        drive = new MecanumDrive(hardwareMap, startPose);

        // Configure PID controller for turing robot
        headingPID = new PIDController(
                Constants.Drive.HEADING_KP,
                Constants.Drive.HEADING_KI,
                Constants.Drive.HEADING_KD
        );

        turnPower = 0;
    }

    @Override
    public void periodic() {
        // Field-centric drive calculations
        double heading = getHeadingRadians();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        // Rotate input vector by -heading
        double fieldX = leftY * cos + leftX * sin;
        double fieldY = leftX * cos - leftY * sin;

        Vector2d fieldCentricInput = new Vector2d(fieldX, fieldY);

        // Calculate turn power
        if (useHeadingLock) {
            turnPower = -headingPID.calculate(headingLockError);
        } else {
            turnPower = -rightX;
        }

        // Apply deadzone
        if (Math.abs(turnPower) < Constants.Drive.DEADZONE && !useHeadingLock) {
            turnPower = 0;
        }

        drive.setDrivePowers(new PoseVelocity2d(fieldCentricInput, turnPower));
        drive.updatePoseEstimate();
    }

    public void setDriveInputs(double leftY, double leftX, double rightX) {
        this.leftY = applyDeadzone(leftY);
        this.leftX = applyDeadzone(leftX);
        this.rightX = applyDeadzone(rightX);
    }

    public void setHeadingLock(boolean enabled, double targetError) {
        this.useHeadingLock = enabled;
        this.headingLockError = targetError;
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public double getHeadingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getHeadingCorrected(Constants.Alliance alliance) {
        double heading = getHeadingRadians();

        double correction = alliance == Constants.Alliance.BLUE ? Math.PI / 2 : -(Math.PI / 2);

        return heading + correction;
    }

    public double getHeadingDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public Pose2d getPose() {
        return drive.localizer.getPose();
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) > Constants.Drive.DEADZONE ? value : 0;
    }

    public Localizer getLocalizer() {
        return drive.localizer;
    }

    public boolean isHealthy() {
        return drive != null && imu != null;
    }

    public void stop() {
        setDriveInputs(0, 0, 0);
    }

    public void updateTelemetry(Telemetry telemetry, Logger logger) {
        Pose2d pose = getPose();

        telemetry.addData(getName() + " Healthy", isHealthy());
        telemetry.addData(getName() + " IMU", getHeadingRadians());
        telemetry.addData(getName() + " X", pose.position.x);
        telemetry.addData(getName() + " Y", pose.position.y);
        telemetry.addData(getName() + " Heading", pose.heading.toDouble());
        telemetry.addData(getName() + " Turn Power", turnPower);
        telemetry.addData(getName() + " Heading Error", headingLockError);
        if (logger != null) {
            logger.put(getName() + " Healthy", isHealthy());
            logger.put(getName() + " X", pose.position.x);
            logger.put(getName() + " Y", pose.position.y);
            logger.put(getName() + " Heading", pose.heading.toDouble());
        }
    }
}