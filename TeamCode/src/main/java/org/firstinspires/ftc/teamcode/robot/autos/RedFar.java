package org.firstinspires.ftc.teamcode.robot.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Constants;

@Autonomous(name="Red Far", group="2025-2026")
public final class RedFar extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // starting pose
        Pose2d beginPose = new Pose2d(66, 16, Math.toRadians(180));

        Pose2d RED_CLOSE = Constants.ShootingPoses.RED_CLOSE;
        Pose2d RED_FAR = Constants.ShootingPoses.RED_FAR;

        double CLOSE_SHOOTER_POWER = Constants.ShootingPower.CLOSE;
        double FAR_SHOOTER_POWER = Constants.ShootingPower.FAR;

        AutoBuilder autoBuilder = new AutoBuilder(hardwareMap, beginPose, AutoBuilder.Alliance.RED);

        waitForStart();

        autoBuilder
                .moveAndShoot(FAR_SHOOTER_POWER, 3, RED_FAR)
                .alignWithArtifacts(21)
                .straightIntake()
                .moveAndShoot(FAR_SHOOTER_POWER, 3, RED_FAR)
                .alignWithArtifacts(22)
                .straightIntake()
                .moveAndShoot(FAR_SHOOTER_POWER, 3, RED_FAR)
                .alignWithArtifacts(21)
                .build();
    }
}
