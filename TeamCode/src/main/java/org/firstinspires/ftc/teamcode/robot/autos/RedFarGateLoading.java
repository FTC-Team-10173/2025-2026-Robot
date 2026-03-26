package org.firstinspires.ftc.teamcode.robot.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Constants;

@Autonomous(name = "Red Far Gate Loading", group = "2025-2026")
public final class RedFarGateLoading extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d startPose = Constants.StartingPoses.RED_FAR;
        Pose2d RED_FAR = Constants.ShootingPoses.RED_FAR;
        Pose2d PARK = Constants.ParkingPoses.RED_FAR;
        Pose2d INTAKE_GATE = Constants.GatePoses.RED_INTAKE;

        double FEED_TIME = Constants.Intake.FEED_TIME_SEC;
        double GATE_TIME = Constants.Intake.GATE_TIME_SEC;

        AutoBuilder autoBuilder = new AutoBuilder(
                hardwareMap,
                startPose,
                Constants.Alliance.RED,
                AutoBuilder.Side.FAR
        )
                .moveAndShoot(FEED_TIME, RED_FAR, 0.5)
                .alignWithArtifacts()
                .straightIntake()
                .moveAndShoot(FEED_TIME, RED_FAR, 0.5)
                .intakeGate(INTAKE_GATE, GATE_TIME)
                .moveAndShoot(FEED_TIME, RED_FAR, 0.5)
                .intakeLoading()
                .moveAndShoot(FEED_TIME, RED_FAR, 0.5)
                .moveToPose(PARK);

        waitForStart();

        if (isStopRequested()) return;

        autoBuilder
                .run()
                .stop();
    }
}
