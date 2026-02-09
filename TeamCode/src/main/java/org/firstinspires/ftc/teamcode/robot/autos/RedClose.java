package org.firstinspires.ftc.teamcode.robot.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Constants;

@Autonomous(name = "Red Close", group = "2025-2026")
public final class RedClose extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d startPose = Constants.StartingPoses.RED_CLOSE;
        Pose2d RED_CLOSE = Constants.ShootingPoses.RED_CLOSE;
        Pose2d PARK = Constants.ParkingPoses.RED_CLOSE;

        double FEED_TIME = Constants.Intake.FEED_TIME_SEC;

        AutoBuilder autoBuilder = new AutoBuilder(
                hardwareMap,
                startPose,
                Constants.Alliance.RED,
                AutoBuilder.Side.CLOSE
        )
                .moveAndShoot(FEED_TIME, RED_CLOSE)
                .alignWithArtifacts()
                .straightIntake()
                .moveAndShoot(FEED_TIME, RED_CLOSE)
                .alignWithArtifacts()
                .straightIntake()
                .moveAndShoot(FEED_TIME, RED_CLOSE)
                .moveToPose(PARK);

        waitForStart();

        if (isStopRequested()) return;

        autoBuilder
                .run()
                .stop();
    }
}
