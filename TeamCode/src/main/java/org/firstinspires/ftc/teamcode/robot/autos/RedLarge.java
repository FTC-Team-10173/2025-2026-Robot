package org.firstinspires.ftc.teamcode.robot.autos;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.LED;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(name="RedLarge", group="2025-2026")
public final class RedLarge extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // starting pose
        Pose2d beginPose = new Pose2d(-54, 48, Math.toRadians(180-45));

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
                                                    .strafeToLinearHeading(new Vector2d(-18, 18), Math.toRadians(180-45))
                                                    .build(),
                                            new SequentialAction(
                                                    robot.shooter.spinUp(0.41, 0.5),
                                                    robot.intake.feed(1, 8500),
                                                    robot.shooter.spinUp(0.0, -1)
                                            )
                                    ),

                                    // align with artifacts
                                    drive.actionBuilder(new Pose2d(-18, 18, Math.toRadians(180-45)))
                                            .strafeToLinearHeading(new Vector2d(-10, 18), Math.toRadians(180-99))
                                            .build(),

                                    // intake artifacts
                                    robot.intake.intake(1),
                                    drive.actionBuilder(new Pose2d(-10, 18, Math.toRadians(180-99)))
                                            .strafeToLinearHeading(new Vector2d(-10, 55), Math.toRadians(180-99), new TranslationalVelConstraint(17.5))
                                            .build(),
                                    robot.intake.intake(0),

                                    // move and shoot
                                    new ParallelAction(
                                            drive.actionBuilder(new Pose2d(-10, 55, Math.toRadians(180-99)))
                                                    .strafeToLinearHeading(new Vector2d(-18, 18), Math.toRadians(180-45))
                                                    .build(),
                                            new SequentialAction(
                                                    robot.shooter.spinUp(0.41, 0.75),
                                                    robot.intake.feed(1, 22500),
                                                    robot.shooter.spinUp(0.0, -1)
                                            )
                                    ),

                                    // Move off white line
                                    drive.actionBuilder(new Pose2d(-18, 18, Math.toRadians(180-40)))
                                            .strafeToLinearHeading(new Vector2d(-48, 18), Math.toRadians(180))
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
