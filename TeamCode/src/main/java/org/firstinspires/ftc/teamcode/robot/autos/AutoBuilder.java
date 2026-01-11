package org.firstinspires.ftc.teamcode.robot.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotState;

import java.util.ArrayList;
import java.util.List;

public class AutoBuilder {
    MecanumDrive drive;
    RobotState robotState;
    Robot robot;
    public Pose2d currentPose;
    int patternID;
    public enum Alliance {
        RED,
        BLUE
    }
    Alliance alliance;

    public enum IntakePose {
        GPP(21, 36),
        PGP(22, 12),
        PPG(23, -12);

        private final int id;
        private final double x;

        IntakePose(int id, double x) {
            this.id = id;
            this.x = x;
        }

        public double getX() {
            return x;
        }

        public int getId() {
            return id;
        }

        public static IntakePose fromId(int id) {
            for (IntakePose p : values()) {
                if (p.id == id) return p;
            }
            throw new IllegalArgumentException("Invalid ShotPose id: " + id);
        }
    }

    List<Action> actions;

    public AutoBuilder(HardwareMap hardwareMap, Pose2d beginPose, Alliance alliance) {
        drive = new MecanumDrive(hardwareMap, beginPose);
        robotState = new RobotState();
        robot = new Robot(hardwareMap, robotState);
        currentPose = beginPose;
        drive.localizer.setPose(beginPose);
        this.alliance = alliance;
        actions = new ArrayList<>();
    }

    public AutoBuilder shoot(double power, int feedDuration) {
        actions.add(
            new SequentialAction(
                robot.shooter.spinUp(power, 1.0),
                robot.intake.feed(1, feedDuration),
                robot.shooter.spinUp(0.0, -1)
            )
        );

        return this;
    }

    public AutoBuilder moveAndShoot(double power, int feedDuration, Pose2d newPose) {
        actions.add(
            new SequentialAction(
                    new ParallelAction(
                            new SequentialAction(
                                    // move to new pose
                                    drive.actionBuilder(currentPose)
                                            .strafeToLinearHeading(new Vector2d(newPose.position.x, newPose.position.y), newPose.heading.toDouble())
                                            .build()
                            ),

                            // spin up shooter while moving
                            robot.shooter.spinUp(power, 1.0)
                    ),

                    // feed artifacts after reaching new pose and spinning up
                    new SequentialAction(
                            robot.intake.feed(1, feedDuration),
                            robot.shooter.spinUp(0.0, -1)
                    )
            )
        );

        currentPose = newPose;

        return this;
    }

    public AutoBuilder straightIntake() {

        // determine strafe Y based on alliance
        double strafeY = (alliance == Alliance.BLUE) ? -55 : 55;

        actions.add(
            new SequentialAction(
                    // start intake
                    robot.intake.intake(1),

                    // strafe through artifacts
                    drive.actionBuilder(currentPose)
                            .strafeToLinearHeading(new Vector2d(currentPose.position.x, strafeY), currentPose.heading.toDouble())
                            .build(),

                    // stop intake
                    robot.intake.intake(0)
            )
        );

        currentPose = new Pose2d(currentPose.position.x, strafeY, currentPose.heading.toDouble());

        return this;
    }

    public AutoBuilder alignWithArtifacts(int patternID) {

        double intakeX = IntakePose.fromId(patternID).getX();

        double intakeY = (alliance == Alliance.BLUE) ? -18 : 18;

        double intakeHeading = (alliance == Alliance.BLUE) ? 270 : 90;

        actions.add(
                new SequentialAction(
                        // strafe to intake position
                        drive.actionBuilder(currentPose)
                                .strafeToLinearHeading(new Vector2d(intakeX, intakeY), Math.toRadians(intakeHeading))
                                .build()
                )
        );

        currentPose = new Pose2d(intakeX, intakeY, Math.toRadians(intakeHeading));

        return this;
    }

    public List<Action> getActions() {
        return actions;
    }

    public void build() {
        Actions.runBlocking(
            new RaceAction(
                new SequentialAction(
                    actions.toArray(new Action[0])
                ),
                robot.led.updateLEDs(),
                robot.shooter.maintainVelocity()
            )
        );

        robot.stopAll();
    }
}
