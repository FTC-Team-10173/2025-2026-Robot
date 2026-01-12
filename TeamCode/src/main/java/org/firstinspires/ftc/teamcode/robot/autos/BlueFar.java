package org.firstinspires.ftc.teamcode.robot.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Constants;

@Autonomous(name = "Blue Far", group = "2025-2026")
public final class BlueFar extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(-54, -48, Math.toRadians(225));
        Pose2d BLUE_FAR = Constants.ShootingPoses.BLUE_FAR;

        double FAR_POWER = Constants.ShootingPower.FAR;

        AutoBuilder autoBuilder = new AutoBuilder(
                hardwareMap,
                startPose,
                AutoBuilder.Alliance.BLUE,
                AutoBuilder.Side.FAR
        );

        waitForStart();

        if (isStopRequested()) return;

        autoBuilder
                .moveAndShoot(FAR_POWER, 3, BLUE_FAR)
                .alignWithArtifactsDeferred()
                .straightIntake()
                .moveAndShoot(FAR_POWER, 3, BLUE_FAR)
                .alignWithArtifactsDeferred()
                .straightIntake()
                .moveAndShoot(FAR_POWER, 3, BLUE_FAR)
                .moveToPose(new Pose2d(60, -36, Math.toRadians(90)))
                .run();

        autoBuilder.stop();
    }
}
