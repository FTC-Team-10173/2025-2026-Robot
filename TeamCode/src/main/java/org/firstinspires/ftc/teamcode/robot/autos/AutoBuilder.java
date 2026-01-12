package org.firstinspires.ftc.teamcode.robot.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotState;

import java.util.ArrayList;
import java.util.List;

public class AutoBuilder {

    /* Config */

    public enum Alliance { RED, BLUE }

    public enum Side { CLOSE(11), FAR(11);
        private final int heading;
        Side(int heading) { this.heading = heading; }
        public int getHeading() { return heading; }
    }
    private final Side side;

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

        public double getX() { return x; }
        public int getId() { return id; }

        public static IntakePose fromId(int id) {
            for (IntakePose p : values()) {
                if (p.id == id) return p;
            }
            throw new IllegalArgumentException("Invalid motif id: " + id);
        }
    }

    /* State */

    private final MecanumDrive drive;
    private final Robot robot;
    private final Alliance alliance;

    private Pose2d currentPose;

    private final List<Action> actions = new ArrayList<>();
    private final List<Integer> availableMotifs = new ArrayList<>(List.of(21, 22, 23));

    private int motifID = -1;

    /* Constructor */
    public AutoBuilder(
            HardwareMap hardwareMap,
            Pose2d startPose,
            Alliance alliance,
            Side side
    ) {
        this.drive = new MecanumDrive(hardwareMap, startPose);
        this.robot = new Robot(hardwareMap, new RobotState());
        this.alliance = alliance;
        this.side = side;
        this.currentPose = startPose;

        drive.localizer.setPose(startPose);
    }

    /* Shooter Actions */
    public AutoBuilder moveAndShoot(double power, double feedTime, Pose2d targetPose) {
        actions.add(new ParallelAction(
                        drive.actionBuilder(currentPose)
                                .strafeToLinearHeading(
                                        new Vector2d(targetPose.position.x, targetPose.position.y),
                                        targetPose.heading.toDouble()
                                )
                                .build(),
                        robot.shooter.spinUp(power, 1.0)
                )
        );

        actions.add(robot.intake.feed(1, feedTime));

        currentPose = targetPose;
        return this;
    }

    public AutoBuilder stopShooter() {
        actions.add(robot.shooter.spinUp(0.0, -1));
        return this;
    }

    /* Intake Actions */
    public AutoBuilder straightIntake() {
        double y = (alliance == Alliance.BLUE) ? -55 : 55;

        actions.add(robot.intake.intake(1));

        actions.add(
                drive.actionBuilder(currentPose)
                        .strafeToLinearHeading(
                                new Vector2d(currentPose.position.x, y),
                                currentPose.heading.toDouble()
                        )
                        .build()
        );

        actions.add(robot.intake.intake(0));

        currentPose = new Pose2d(currentPose.position.x, y, currentPose.heading.toDouble());
        return this;
    }

    /* Motif Actions */
    public AutoBuilder moveToMotif(Pose2d shootPose) {
        int readHeading = side.getHeading();
        readHeading = (alliance == AutoBuilder.Alliance.BLUE) ? (180 - readHeading) : (180 + readHeading);

        actions.add(new ParallelAction(
                drive.actionBuilder(currentPose)
                        .strafeToLinearHeading(new Vector2d(shootPose.position.x, shootPose.position.y), Math.toRadians(readHeading))
                        .build(),
                readMotif()
        ));
        return this;
    }

    public Action readMotif() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.limelight.periodic();
                int detected = robot.limelight.getMotifID();

                if (detected != -1) {
                    motifID = detected;
                    packet.put("Motif", motifID);
                    return false;
                }

                packet.put("Motif", "searching");
                return true;
            }
        };
    }

    /* Movement Actions */

    public AutoBuilder alignWithArtifactsDeferred() {
        actions.add(new Action() {

            private Action inner = null;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                // Build ONCE
                if (inner == null) {

                    int selected;

                    // Prefer vision result
                    if (motifID != -1) {
                        selected = motifID;
                        packet.put("Align", "using vision motif " + selected);
                        motifID = -1;
                    }
                    // Otherwise use side-based rule
                    else {
                        if (availableMotifs.isEmpty()) {
                            throw new IllegalStateException("No motifs remaining");
                        }

                        selected = (side == Side.CLOSE)
                                ? availableMotifs.stream().max(Integer::compare).get()
                                : availableMotifs.stream().min(Integer::compare).get();

                        packet.put("Align", "fallback side=" + side + " motif=" + selected);
                    }

                    availableMotifs.remove(Integer.valueOf(selected));

                    double x = IntakePose.fromId(selected).getX();
                    double y = (alliance == Alliance.BLUE) ? -18 : 18;
                    double heading = Math.toRadians(
                            (alliance == Alliance.BLUE) ? 270 : 90
                    );

                    inner = drive.actionBuilder(currentPose)
                            .strafeToLinearHeading(
                                    new Vector2d(x, y),
                                    heading
                            )
                            .build();

                    currentPose = new Pose2d(x, y, heading);
                }

                return inner.run(packet);
            }
        });

        return this;
    }

    public AutoBuilder moveToPose(Pose2d targetPose) {
        actions.add(
                drive.actionBuilder(currentPose)
                        .strafeToLinearHeading(
                                new Vector2d(targetPose.position.x, targetPose.position.y),
                                targetPose.heading.toDouble()
                        )
                        .build()
        );
        return this;
    }

    /* Finalization */

    public Action build() {
        return new RaceAction(
                new SequentialAction(actions),
                robot.shooter.maintainVelocity(),
                robot.led.updateLEDs()
        );
    }

    public AutoBuilder run() {
        Actions.runBlocking(build());
        return  this;
    }

    public void stop() {
        robot.stopAll();
    }
}
