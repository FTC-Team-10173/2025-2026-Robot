package org.firstinspires.ftc.teamcode.robot.autos.reference;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.LED;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Disabled
@Autonomous(name="BlueLarge", group="Roadrunner Autos")
public final class BlueLargeReference extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(-54, -48, Math.toRadians(225));

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            LED led = new LED(hardwareMap);
            Shooter shooter = new Shooter(hardwareMap, led);
            Intake intake = new Intake(hardwareMap);

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
                                                    shooter.spinUp(0.43, 0.5),
                                                    intake.feed(1, 10000),
                                                    shooter.spinUp(0.0, -1)
                                            )
                                    ),

                                    // align with artifacts
                                    drive.actionBuilder(new Pose2d(-18, -18, Math.toRadians(225)))
                                            .strafeToLinearHeading(new Vector2d(-12, -18), Math.toRadians(270))
                                            .build(),

                                    // intake artifacts
                                    intake.intake(1),
                                    drive.actionBuilder(new Pose2d(-12, -18, Math.toRadians(270)))
                                            .strafeToLinearHeading(new Vector2d(-12, -54), Math.toRadians(270), new TranslationalVelConstraint(25))
                                            .build(),
                                    intake.intake(0),

                                    // move and shoot
                                    new ParallelAction(
                                            drive.actionBuilder(new Pose2d(-12, -54, Math.toRadians(270)))
                                                    .strafeToLinearHeading(new Vector2d(-18, -18), Math.toRadians(225))
                                                    .build(),
                                            new SequentialAction(
                                                    shooter.spinUp(0.43, 0.5),
                                                    intake.feed(1, 10000),
                                                    shooter.spinUp(0.0, -1)
                                            )
                                    ),

                                    // align with artifacts
                                    drive.actionBuilder(new Pose2d(-18, -18, Math.toRadians(225)))
                                            .strafeToLinearHeading(new Vector2d(12, -18), Math.toRadians(270))
                                            .build(),

                                    // intake artifacts
                                    intake.intake(1),
                                    drive.actionBuilder(new Pose2d(12, -18, Math.toRadians(270)))
                                            .strafeToLinearHeading(new Vector2d(12, -54), Math.toRadians(270), new TranslationalVelConstraint(25))
                                            .build(),
                                    intake.intake(0),

                                    // move and shoot
                                    new ParallelAction(
                                            drive.actionBuilder(new Pose2d(12, -54, Math.toRadians(270)))
                                                    .strafeToLinearHeading(new Vector2d(-18, -18), Math.toRadians(225))
                                                    .build(),
                                            new SequentialAction(
                                                    shooter.spinUp(0.43, 0.5),
                                                    intake.feed(1, 10000),
                                                    shooter.spinUp(0.0, -1)
                                            )
                                    ),

                                    // align with artifacts
                                    drive.actionBuilder(new Pose2d(-18, -18, Math.toRadians(225)))
                                            .strafeToLinearHeading(new Vector2d(36, -18), Math.toRadians(270))
                                            .build(),

                                    // intake artifacts
                                    intake.intake(1),
                                    drive.actionBuilder(new Pose2d(36, -18, Math.toRadians(270)))
                                            .strafeToLinearHeading(new Vector2d(36, -54), Math.toRadians(270), new TranslationalVelConstraint(25))
                                            .build(),
                                    intake.intake(0),

                                    // move and shoot
                                    new ParallelAction(
                                            drive.actionBuilder(new Pose2d(36, -54, Math.toRadians(270)))
                                                    .strafeToLinearHeading(new Vector2d(-18, -18), Math.toRadians(225))
                                                    .build(),
                                            new SequentialAction(
                                                    shooter.spinUp(0.43, 0.5),
                                                    intake.feed(1, 10000),
                                                    shooter.spinUp(0.0, -1)
                                            )
                                    )
                            ),
                            shooter.maintainVelocity()
                    )
            );
        } else {
            throw new RuntimeException();
        }
    }
}
