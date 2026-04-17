package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.MyConstants;
import org.firstinspires.ftc.teamcode.robot.ShooterMath;

import java.lang.Math;

public class Drive extends SubsystemBase {
    public final Follower follower;
    private final IMU imu;
    private double leftY = 0;
    private double leftX = 0;
    private double rightX = 0;
    private double headingLockError = 0;
    private double goalHeadingError = 0;

    public Drive(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                        )
                )
        );

        follower = Constants.createFollower(hardwareMap);
        follower.update();
        follower.startTeleOpDrive();

        resetYaw();
    }

    @Override
    public void periodic() {
        follower.setTeleOpDrive(
                leftX,
                leftY,
                rightX,
                false
        );
    }

    public void setDriveInputs(double leftY, double leftX, double rightX) {
        this.leftY = applyDeadzone(leftY);
        this.leftX = applyDeadzone(leftX);
        this.rightX = applyDeadzone(rightX);
    }

    public void updateGoalHeadingError(Pose pose) {
        goalHeadingError = ShooterMath.getGoalError(pose);
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public double getHeadingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getHeadingCorrected() {
        double heading = getHeadingRadians();

        double correction = Alliance.isBlue() ? -(Math.PI / 2) : Math.PI / 2;

        return heading + correction;
    }

    public double getHeadingDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public Pose getPose() {
        return follower.poseTracker.getPose();
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) > MyConstants.Drive.DEADZONE ? value : 0;
    }

    public PoseTracker getLocalizer() {
        return follower.poseTracker;
    }

    public boolean isHealthy() {
        return follower != null && imu != null;
    }

    public void stop() {
        setDriveInputs(0, 0, 0);
    }

    public void updateTelemetry(Telemetry telemetry) {
        Pose pose = getPose();

        telemetry.addData(getName() + " Healthy", isHealthy());
        telemetry.addData(getName() + " IMU", getHeadingRadians());
        telemetry.addData(getName() + " X", pose.getX());
        telemetry.addData(getName() + " Y", pose.getY());
        telemetry.addData(getName() + " Heading", pose.getHeading());
        telemetry.addData(getName() + " Heading Error", headingLockError);
        telemetry.addData(getName() + " goalHeadingError", goalHeadingError);
    }
}