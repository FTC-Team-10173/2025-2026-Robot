package org.firstinspires.ftc.teamcode.PedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.07)
            .forwardZeroPowerAcceleration(-39.26103471013033)
            .lateralZeroPowerAcceleration(-61.21165207406077)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.125,0.0,0.0075,0.6,0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.1, 0.01))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0, 0))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.14140164892325033, 0.0013280906800557987))
            .centripetalScaling(0);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelIMULocalizer(localizerConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(59.172193577239077)
            .yVelocity(41.08005361188956);

    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(0.002949098781967676)
            .strafeTicksToInches(-0.0029930411182586702)
            .turnTicksToInches(0.00191817799558734203)
            .leftPodY(7.625)
            .rightPodY(-7.625)
            .strafePodX(-5.25)
            .leftEncoder_HardwareMapName("frontLeft")
            .rightEncoder_HardwareMapName("frontRight")
            .strafeEncoder_HardwareMapName("backRight")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
}
