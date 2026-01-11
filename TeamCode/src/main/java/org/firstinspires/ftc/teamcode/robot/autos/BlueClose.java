package org.firstinspires.ftc.teamcode.robot.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Constants;

@Autonomous(name="Blue Close", group="2025-2026")
public final class BlueClose extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // starting pose
        Pose2d beginPose = new Pose2d(-54, -48, Math.toRadians(225));

        Pose2d BLUE_CLOSE = Constants.ShootingPoses.BLUE_CLOSE;
        Pose2d BLUE_FAR = Constants.ShootingPoses.BLUE_FAR;

        double CLOSE_SHOOTER_POWER = Constants.ShootingPower.CLOSE;
        double FAR_SHOOTER_POWER = Constants.ShootingPower.FAR;

        AutoBuilder autoBuilder = new AutoBuilder(hardwareMap, beginPose, AutoBuilder.Alliance.BLUE);

        waitForStart();

        autoBuilder
                .moveAndShoot(CLOSE_SHOOTER_POWER, 3, BLUE_CLOSE)
                .alignWithArtifacts(23)
                .straightIntake()
                .moveAndShoot(CLOSE_SHOOTER_POWER, 3, BLUE_CLOSE)
                .alignWithArtifacts(22)
                .straightIntake()
                .moveAndShoot(CLOSE_SHOOTER_POWER, 3, BLUE_CLOSE)
                .alignWithArtifacts(22)
                .build();
    }
}
