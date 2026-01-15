package org.firstinspires.ftc.teamcode.robot.autos.test;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.autos.AutoBuilder;

@Disabled
@Autonomous(name = "GateTest Auto", group = "2025-2026")
public final class GateTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d startPose = Constants.StartingPoses.BLUE_CLOSE;
        Pose2d BLUE_CLOSE = Constants.ShootingPoses.BLUE_CLOSE;
        Pose2d OPEN_GATE = Constants.GatePoses.BLUE_OPEN;
        Pose2d INTAKE_GATE = Constants.GatePoses.BLUE_INTAKE;

        double CLOSE_POWER = Constants.ShootingPower.CLOSE;

        double FEED_TIME = Constants.Intake.FEED_TIME_SEC;

        AutoBuilder autoBuilder = new AutoBuilder(
                hardwareMap,
                startPose,
                AutoBuilder.Alliance.BLUE,
                AutoBuilder.Side.CLOSE
        )
                .moveAndShoot(CLOSE_POWER, FEED_TIME, BLUE_CLOSE)
                .intakeGate(INTAKE_GATE, FEED_TIME)
                .moveAndShoot(CLOSE_POWER, FEED_TIME, BLUE_CLOSE)
                .openGate(OPEN_GATE);

        waitForStart();

        if (isStopRequested()) return;

        autoBuilder
                .run()
                .stop();
    }
}
