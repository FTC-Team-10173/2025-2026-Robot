package org.firstinspires.ftc.teamcode.robot.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Constants;

@Autonomous(name = "Blue Far", group = "2025-2026")
public final class BlueFar extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d startPose = Constants.StartingPoses.BLUE_FAR;
        Pose2d BLUE_FAR = Constants.ShootingPoses.BLUE_FAR;
        Pose2d PARK = Constants.ParkingPoses.BLUE_FAR;

        double FAR_POWER = Constants.ShootingPower.FAR;

        double FEED_TIME = Constants.Intake.FEED_TIME_SEC;

        AutoBuilder autoBuilder = new AutoBuilder(
                hardwareMap,
                startPose,
                AutoBuilder.Alliance.BLUE,
                AutoBuilder.Side.FAR
        )
                .moveAndShoot(FAR_POWER, FEED_TIME, BLUE_FAR)
                .alignWithArtifacts()
                .straightIntake()
                .moveAndShoot(FAR_POWER, FEED_TIME, BLUE_FAR)
                .alignWithArtifacts()
                .straightIntake()
                .moveAndShoot(FAR_POWER, FEED_TIME, BLUE_FAR)
                .moveToPose(PARK);

        waitForStart();

        if (isStopRequested()) return;

        autoBuilder
                .run()
                .stop();
    }
}
