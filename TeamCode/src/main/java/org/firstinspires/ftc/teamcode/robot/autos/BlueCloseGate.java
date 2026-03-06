package org.firstinspires.ftc.teamcode.robot.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Constants;

@Autonomous(name = "Blue Close Gate", group = "2025-2026")
public final class BlueCloseGate extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d startPose = Constants.StartingPoses.BLUE_CLOSE;
        Pose2d BLUE_CLOSE_BACK = Constants.ShootingPoses.BLUE_CLOSE_BACK;
        Pose2d BLUE_CLOSE = Constants.ShootingPoses.BLUE_CLOSE;
        Pose2d INTAKE_GATE = Constants.GatePoses.BLUE_INTAKE;
        Pose2d PARK = Constants.ParkingPoses.BLUE_CLOSE;

        double FEED_TIME = Constants.Intake.FEED_TIME_SEC;
        double GATE_TIME = Constants.Intake.GATE_TIME_SEC;

        AutoBuilder autoBuilder = new AutoBuilder(
                hardwareMap,
                startPose,
                Constants.Alliance.BLUE,
                AutoBuilder.Side.CLOSE
        )
                .moveAndShoot(FEED_TIME, BLUE_CLOSE)
                .alignWithArtifacts(22)
                .straightIntake()
                .moveAndShoot(FEED_TIME, BLUE_CLOSE_BACK)
                .intakeGate(INTAKE_GATE, GATE_TIME)
                .moveAndShoot(FEED_TIME, BLUE_CLOSE_BACK)
                .intakeGate(INTAKE_GATE, GATE_TIME)
                .moveAndShoot(FEED_TIME, BLUE_CLOSE_BACK)
                .moveToPose(PARK);

        waitForStart();

        if (isStopRequested()) return;

        autoBuilder
                .run()
                .stop();
    }
}
