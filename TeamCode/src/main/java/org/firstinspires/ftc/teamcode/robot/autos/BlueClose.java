package org.firstinspires.ftc.teamcode.robot.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Constants;

@Autonomous(name = "Blue Close", group = "2025-2026")
public final class BlueClose extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d startPose = Constants.StartingPoses.BLUE_CLOSE;
        Pose2d BLUE_CLOSE = Constants.ShootingPoses.BLUE_CLOSE;
        Pose2d PARK = Constants.ParkingPoses.BLUE_CLOSE;

        double FEED_TIME = Constants.Intake.FEED_TIME_SEC;

        AutoBuilder autoBuilder = new AutoBuilder(
                hardwareMap,
                startPose,
                AutoBuilder.Alliance.BLUE,
                AutoBuilder.Side.CLOSE
        )
                .moveAndShoot(FEED_TIME, BLUE_CLOSE)
                .alignWithArtifacts()
                .straightIntake()
                .moveAndShoot(FEED_TIME, BLUE_CLOSE)
                .alignWithArtifacts()
                .straightIntake()
                .moveAndShoot(FEED_TIME, BLUE_CLOSE)
                .moveToPose(PARK);

        waitForStart();

        if (isStopRequested()) return;

        autoBuilder
                .run()
                .stop();
    }
}
