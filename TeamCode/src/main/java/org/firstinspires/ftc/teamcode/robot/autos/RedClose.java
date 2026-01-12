package org.firstinspires.ftc.teamcode.robot.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Constants;

@Autonomous(name = "Red Close", group = "2025-2026")
public final class RedClose extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(-54, -48, Math.toRadians(225));
        Pose2d RED_CLOSE = Constants.ShootingPoses.RED_CLOSE;

        double CLOSE_POWER = Constants.ShootingPower.CLOSE;

        AutoBuilder autoBuilder = new AutoBuilder(
                hardwareMap,
                startPose,
                AutoBuilder.Alliance.RED,
                AutoBuilder.Side.CLOSE
        );

        waitForStart();

        if (isStopRequested()) return;

        autoBuilder
                .moveAndShoot(CLOSE_POWER, 3, RED_CLOSE)
                .alignWithArtifactsDeferred()
                .straightIntake()
                .moveAndShoot(CLOSE_POWER, 3, RED_CLOSE)
                .alignWithArtifactsDeferred()
                .straightIntake()
                .moveAndShoot(CLOSE_POWER, 3, RED_CLOSE)
                .moveToPose(new Pose2d(-60, 12, Math.toRadians(270)))
                .run();

        autoBuilder.stop();
    }
}
