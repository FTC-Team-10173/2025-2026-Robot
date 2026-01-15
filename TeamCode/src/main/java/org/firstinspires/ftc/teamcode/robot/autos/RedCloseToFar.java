package org.firstinspires.ftc.teamcode.robot.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Constants;

@Autonomous(name = "Red Close To Far", group = "2025-2026")
public final class RedCloseToFar extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d startPose = Constants.StartingPoses.RED_CLOSE;
        Pose2d RED_CLOSE = Constants.ShootingPoses.RED_CLOSE;
        Pose2d RED_FAR = Constants.ShootingPoses.RED_FAR;
        Pose2d PARK = Constants.ParkingPoses.RED_FAR;

        double CLOSE_POWER = Constants.ShootingPower.CLOSE;
        double FAR_POWER = Constants.ShootingPower.FAR;

        double FEED_TIME = Constants.Intake.FEED_TIME_SEC;

        AutoBuilder autoBuilder = new AutoBuilder(
                hardwareMap,
                startPose,
                AutoBuilder.Alliance.RED,
                AutoBuilder.Side.CLOSE
        )
                .moveAndShoot(CLOSE_POWER, FEED_TIME, RED_CLOSE)
                .alignWithArtifacts()
                .straightIntake()
                .moveAndShoot(CLOSE_POWER, FEED_TIME, RED_CLOSE)
                .alignWithArtifacts()
                .straightIntake()
                .moveAndShoot(CLOSE_POWER, FEED_TIME, RED_CLOSE)
                .alignWithArtifacts()
                .straightIntake()
                .moveAndShoot(FAR_POWER, FEED_TIME, RED_FAR)
                .moveToPose(PARK);

        waitForStart();

        if (isStopRequested()) return;

        autoBuilder
                .run()
                .stop();
    }
}
