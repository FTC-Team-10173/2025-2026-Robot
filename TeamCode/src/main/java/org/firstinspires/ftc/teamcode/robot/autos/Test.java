package org.firstinspires.ftc.teamcode.robot.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Constants;

@Autonomous(name = "Test Auto", group = "2025-2026")
public final class Test extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(-54, -48, Math.toRadians(225));
        Pose2d BLUE_CLOSE = Constants.ShootingPoses.BLUE_CLOSE;

        double CLOSE_POWER = Constants.ShootingPower.CLOSE;

        AutoBuilder autoBuilder = new AutoBuilder(
                hardwareMap,
                startPose,
                AutoBuilder.Alliance.BLUE,
                AutoBuilder.Side.CLOSE
        );

        waitForStart();

        if (isStopRequested()) return;

        autoBuilder
                .moveAndShoot(CLOSE_POWER, 3, BLUE_CLOSE)
                .moveToMotif(BLUE_CLOSE)
                .alignWithArtifactsDeferred()
                .straightIntake()
                .moveAndShoot(CLOSE_POWER, 3, BLUE_CLOSE)
                .alignWithArtifactsDeferred()
                .straightIntake()
                .moveAndShoot(CLOSE_POWER, 3, BLUE_CLOSE)
                .alignWithArtifactsDeferred()
                .straightIntake()
                .moveAndShoot(CLOSE_POWER, 3, BLUE_CLOSE)
                .moveToPose(new Pose2d(-60, -12, Math.toRadians(90)))
                .run();

        autoBuilder.stop();
    }
}
