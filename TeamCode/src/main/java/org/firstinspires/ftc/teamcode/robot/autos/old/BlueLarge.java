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
@Autonomous(name="BlueLarge", group="2025-2026")
public final class BlueLarge extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // starting pose
        Pose2d beginPose = new Pose2d(-54, -48, Math.toRadians(225));

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
                                                    .strafeToLinearHeading(new Vector2d(-18, -18), Math.toRadians(225))
                                                    .build(),
                                            new SequentialAction(
                                                    robot.shooter.spinUp(0.41, 0.5),
                                                    robot.intake.feed(1, 8500),
                                                    robot.shooter.spinUp(0.0, -1)
                                            )
                                    ),

                                    // align with artifacts
                                    drive.actionBuilder(new Pose2d(-18, -18, Math.toRadians(225)))
                                            .strafeToLinearHeading(new Vector2d(-7, -18), Math.toRadians(279))
                                            .build(),

                                    // intake artifacts
                                    robot.intake.intake(1),
                                    drive.actionBuilder(new Pose2d(-7, -18, Math.toRadians(279)))
                                            .strafeToLinearHeading(new Vector2d(-7, -55), Math.toRadians(279), new TranslationalVelConstraint(17.5))
                                            .build(),
                                    robot.intake.intake(0),

                                    // move and shoot
                                    new ParallelAction(
                                            drive.actionBuilder(new Pose2d(-12, -55, Math.toRadians(279)))
                                                    .strafeToLinearHeading(new Vector2d(-18, -18), Math.toRadians(215))
                                                    .build(),
                                            new SequentialAction(
                                                    robot.shooter.spinUp(0.41, 0.75),
                                                    robot.intake.feed(1, 22500),
                                                    robot.shooter.spinUp(0.0, -1)
                                            )
                                    ),

                                    // Move off white line
                                    drive.actionBuilder(new Pose2d(-18, -18, Math.toRadians(215)))
                                            .strafeToLinearHeading(new Vector2d(-48, -18), Math.toRadians(180))
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
