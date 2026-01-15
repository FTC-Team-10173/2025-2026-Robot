package org.firstinspires.ftc.teamcode.robot.RedCloseToFar.old;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotState;
import org.firstinspires.ftc.teamcode.Roadrunner.tuning.TuningOpModes;

@Disabled
@Autonomous(name="BlueSmall", group="2025-2026")
public final class BlueSmall extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // starting pose
        Pose2d beginPose = new Pose2d(64, -16, Math.toRadians(180));

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            RobotState robotState = new RobotState();
            Robot robot = new Robot(hardwareMap, robotState);

            waitForStart();

            Actions.runBlocking(
                    new ParallelAction(
                            new SequentialAction(
                                    // move and shoot
                                    new ParallelAction(
                                            drive.actionBuilder(beginPose)
                                                    .strafeToLinearHeading(new Vector2d(52, -16), Math.toRadians(204))
                                                    .build(),
                                            new SequentialAction(
                                                    robot.shooter.spinUp(0.49, 0.5),
                                                    robot.intake.feed(1, 10000),
                                                    robot.shooter.spinUp(0.0, -1)
                                            )
                                    ),

                                    // align with artifacts
                                    drive.actionBuilder(new Pose2d(52, -16, Math.toRadians(204)))
                                            .strafeToLinearHeading(new Vector2d(36, -18), Math.toRadians(265))
                                            .build(),

                                    // intake artifacts
                                    robot.intake.intake(1),
                                    drive.actionBuilder(new Pose2d(36, -18, Math.toRadians(265)))
                                            .strafeToLinearHeading(new Vector2d(36, -60), Math.toRadians(270), new TranslationalVelConstraint(25))
                                            .build(),
                                    robot.intake.intake(0),

                                    // move and shoot
                                    new ParallelAction(
                                            drive.actionBuilder(new Pose2d(36, -60, Math.toRadians(270)))
                                                    .strafeToLinearHeading(new Vector2d(52, -14), Math.toRadians(189))
                                                    .build(),
                                            new SequentialAction(
                                                    robot.shooter.spinUp(0.49, 0.75),
                                                    robot.intake.feed(1, 22500),
                                                    robot.shooter.spinUp(0.0, -1)
                                            )
                                    ),

                                    // Move off white line
                                    drive.actionBuilder(new Pose2d(52, -14, Math.toRadians(189)))
                                            .strafeToLinearHeading(new Vector2d(40, -14), Math.toRadians(189))
                                            .build()
                            ),
                            robot.shooter.maintainVelocity()
                    )
            );
        } else {
            throw new RuntimeException();
        }
    }
}
