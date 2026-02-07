package org.firstinspires.ftc.teamcode.robot.autos;

import static org.firstinspires.ftc.teamcode.robot.Constants.BlackBoard;
import static org.firstinspires.ftc.teamcode.robot.Constants.Keys.ALLIANCE;
import static org.firstinspires.ftc.teamcode.robot.Constants.Keys.POSE;

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
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.LED;
import org.firstinspires.ftc.teamcode.robot.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;

public class AutoBuilder {

    /* Config */

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

    private final Robot robot;

    private final MecanumDrive drive;
    private final Shooter shooter;
    private final Intake intake;
    private final LED led;
    private final Limelight limelight;

    private final Constants.Alliance alliance;
    private final List<Action> actions = new ArrayList<>();
    private final List<Integer> availableMotifs = new ArrayList<>(List.of(21, 22, 23));
    private int motifID = -1;

    /* Constructor */

    public AutoBuilder(
            HardwareMap hardwareMap,
            Pose2d startPose,
            Constants.Alliance alliance,
            Side side
    ) {
        this.robot = new Robot(hardwareMap, alliance, startPose);

        this.drive = robot.getDrive().drive;
        this.shooter = robot.getShooter();
        this.intake = robot.getIntake();
        this.led = robot.getLed();
        this.limelight = robot.getLimelight();

        this.alliance = alliance;
        this.side = side;
        drive.localizer.setPose(startPose);
        BlackBoard.put(ALLIANCE, alliance);
    }

    /* Shooter Actions */

    public AutoBuilder moveAndShoot(double feedTime, Pose2d targetPose) {
        actions.add(new Action() {

            private Action inner = null;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                // Build ONCE
                if (inner == null) {

                    // get current pose from localizer
                    Pose2d currentPose = drive.localizer.getPose();

                    inner = new ParallelAction(
                            drive.actionBuilder(currentPose)
                                    .strafeToLinearHeading(
                                            new Vector2d(targetPose.position.x, targetPose.position.y),
                                            targetPose.heading.toDouble()
                                    )
                                    .build(),
                            robot.setPower(targetPose),
                            shooter.startShooter(),
                            intake.open()
                    );
                }

                return inner.run(packet);
            }
        });

        actions.add(intake.feed(1.0, feedTime));

        return this;
    }

    public AutoBuilder stopShooter() {
        actions.add(shooter.stopShooter());
        return this;
    }

    /* Intake Actions */

    public AutoBuilder straightIntake() {
        double y = (alliance == Constants.Alliance.BLUE) ? -55 : 55;

        actions.add(intake.intake(1));

        actions.add(new Action() {

            private Action inner = null;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                // Build ONCE
                if (inner == null) {

                    Pose2d currentPose = drive.localizer.getPose();

                    inner = drive.actionBuilder(currentPose)
                            .strafeToLinearHeading(
                                    new Vector2d(currentPose.position.x, y),
                                    currentPose.heading.toDouble()
                            )
                            .build();
                }

                return inner.run(packet);
            }
        });

        actions.add(intake.intake(0));

        return this;
    }

    public AutoBuilder straightIntake(boolean returnToStart) {
        double y = (alliance == Constants.Alliance.BLUE) ? -55 : 55;

        actions.add(intake.intake(1));

        actions.add(new Action() {

            private Action inner = null;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                // Build ONCE
                if (inner == null) {

                    Pose2d currentPose = drive.localizer.getPose();

                    if (motifID == 22) {
                        inner = drive.actionBuilder(currentPose)
                                .strafeToLinearHeading(
                                        new Vector2d(currentPose.position.x, y),
                                        currentPose.heading.toDouble()
                                )
                                .strafeToLinearHeading(
                                        new Vector2d(currentPose.position.x, currentPose.position.y),
                                        currentPose.heading.toDouble()
                                )
                                .build();
                    }
                    else {
                        inner = drive.actionBuilder(currentPose)
                                .strafeToLinearHeading(
                                        new Vector2d(currentPose.position.x, y),
                                        currentPose.heading.toDouble()
                                )
                                .build();
                    }
                }

                return inner.run(packet);
            }
        });

        actions.add(intake.intake(0));

        return this;
    }

    /* Gate Actions */

    public AutoBuilder openGate(Pose2d gatePose) {
        actions.add(intake.intake(1));

        double offsetY = (alliance == Constants.Alliance.BLUE) ? 6 : -6;

        actions.add(new Action() {
            private Action inner = null;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                // Build ONCE
                if (inner == null) {

                    Pose2d currentPose = drive.localizer.getPose();

                    inner = drive.actionBuilder(currentPose)
                            .strafeToLinearHeading(new Vector2d(gatePose.position.x, gatePose.position.y + offsetY), Math.toRadians(gatePose.heading.toDouble()))
                            .strafeToLinearHeading(new Vector2d(gatePose.position.x, gatePose.position.y), Math.toRadians(gatePose.heading.toDouble()))
                            .build();
                }

                return inner.run(packet);
            }
        });

        actions.add(wait(0.5));

        return this;
    }

    public AutoBuilder intakeGate(Pose2d gatePose, double intakeTime) {
        actions.add(intake.intake(1));

        double offsetY = (alliance == Constants.Alliance.BLUE) ? 6 : -6;

        actions.add(new Action() {
            private Action inner = null;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                // Build ONCE
                if (inner == null) {

                    Pose2d currentPose = drive.localizer.getPose();

                    inner = drive.actionBuilder(currentPose)
                            .strafeToLinearHeading(new Vector2d(gatePose.position.x, gatePose.position.y + offsetY), Math.toRadians(gatePose.heading.toDouble()))
                            .strafeToLinearHeading(new Vector2d(gatePose.position.x, gatePose.position.y), Math.toRadians(gatePose.heading.toDouble()))
                            .build();
                }

                return inner.run(packet);
            }
        });

        actions.add(intake.intake(1, intakeTime));

        return this;
    }

    /* Movement Actions */

    public AutoBuilder alignWithArtifacts() {
        actions.add(new Action() {

            private Action inner = null;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                // Build ONCE
                if (inner == null) {

                    int selected;

                    if (availableMotifs.isEmpty()) {
                        throw new IllegalStateException("No motifs remaining");
                    }

                    selected = (side == Side.CLOSE)
                            ? availableMotifs.stream().max(Integer::compare).get()
                            : availableMotifs.stream().min(Integer::compare).get();

                    packet.put("Align", "aligning with motif " + selected);

                    availableMotifs.remove(Integer.valueOf(selected));

                    double x = IntakePose.fromId(selected).getX();
                    double y = (alliance == Constants.Alliance.BLUE) ? -18 : 18;
                    double heading = Math.toRadians(
                            (alliance == Constants.Alliance.BLUE) ? 270 : 90
                    );

                    Pose2d currentPose = drive.localizer.getPose();

                    inner = drive.actionBuilder(currentPose)
                            .strafeToLinearHeading(
                                    new Vector2d(x, y),
                                    heading
                            )
                            .build();
                }

                return inner.run(packet);
            }
        });

        return this;
    }

    public AutoBuilder alignWithArtifacts(boolean usingMotif) {
        actions.add(new Action() {

            private Action inner = null;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                // Build ONCE
                if (inner == null) {

                    int selected;

                    selected = motifID;
                    packet.put("Align", "using vision motif " + selected);

                    availableMotifs.remove(Integer.valueOf(selected));

                    double x = IntakePose.fromId(selected).getX();
                    double y = (alliance == Constants.Alliance.BLUE) ? -18 : 18;
                    double heading = Math.toRadians(
                            (alliance == Constants.Alliance.BLUE) ? 270 : 90
                    );

                    Pose2d currentPose = drive.localizer.getPose();

                    inner = drive.actionBuilder(currentPose)
                            .strafeToLinearHeading(
                                    new Vector2d(x, y),
                                    heading
                            )
                            .build();
                }

                return inner.run(packet);
            }
        });

        return this;
    }

    public AutoBuilder alignWithArtifacts(int motifID) {
        actions.add(new Action() {

            private Action inner = null;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                // Build ONCE
                if (inner == null) {

                    int selected;

                    if (availableMotifs.isEmpty()) {
                        throw new IllegalStateException("No motifs remaining");
                    }

                    packet.put("Align", "aligning with motif " + motifID);

                    availableMotifs.remove(Integer.valueOf(motifID));

                    double x = IntakePose.fromId(motifID).getX();
                    double y = (alliance == Constants.Alliance.BLUE) ? -18 : 18;
                    double heading = Math.toRadians(
                            (alliance == Constants.Alliance.BLUE) ? 270 : 90
                    );

                    Pose2d currentPose = drive.localizer.getPose();

                    inner = drive.actionBuilder(currentPose)
                            .strafeToLinearHeading(
                                    new Vector2d(x, y),
                                    heading
                            )
                            .build();
                }

                return inner.run(packet);
            }
        });

        return this;
    }

    public AutoBuilder intakeLoading() {
        actions.add(new Action() {
            private Action inner = null;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                // Build ONCE
                if (inner == null) {

                    double y = (alliance == Constants.Alliance.BLUE) ? -66 : 66;
                    double heading = Math.toRadians(
                            (alliance == Constants.Alliance.BLUE) ? 345 : 15
                    );

                    Pose2d currentPose = drive.localizer.getPose();

                    inner = drive.actionBuilder(currentPose)
                                .strafeToLinearHeading(
                                        new Vector2d(48, y),
                                        0
                                )
                                .build();
                }

                return inner.run(packet);
            }
        });

        actions.add(intake.intake(1));

        actions.add(new Action() {
            private Action inner = null;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                // Build ONCE
                if (inner == null) {

                    double y = (alliance == Constants.Alliance.BLUE) ? -66 : 66;
                    double heading = Math.toRadians(
                            (alliance == Constants.Alliance.BLUE) ? 345 : 15
                    );

                    Pose2d currentPose = drive.localizer.getPose();

                    inner = drive.actionBuilder(currentPose)
                                .strafeToLinearHeading(
                                        new Vector2d(66, y),
                                        0
                                )
                                .build();
                }

                return inner.run(packet);
            }
        });

        actions.add(new Action() {
            private Action inner = null;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                // Build ONCE
                if (inner == null) {

                    double y = (alliance == Constants.Alliance.BLUE) ? -66 : 66;
                    double heading = Math.toRadians(
                            (alliance == Constants.Alliance.BLUE) ? 270 : 90
                    );

                    Pose2d currentPose = drive.localizer.getPose();

                    inner = drive.actionBuilder(currentPose)
                            .strafeToLinearHeading(
                                    new Vector2d(66, y),
                                    heading
                            )
                            .build();
                }

                return inner.run(packet);
            }
        });

        actions.add(wait(0.5));
        actions.add(intake.intake(0));

        return this;
    }

    public AutoBuilder moveToPose(Pose2d targetPose) {
        actions.add(new Action() {
            private Action inner = null;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                // Build ONCE
                if (inner == null) {

                    Pose2d currentPose = drive.localizer.getPose();

                    inner = drive.actionBuilder(currentPose)
                            .strafeToLinearHeading(
                                    new Vector2d(targetPose.position.x, targetPose.position.y),
                                    targetPose.heading.toDouble()
                            )
                            .build();
                }

                return inner.run(packet);
            }
        });

        return this;
    }

    /* Utility */

    public Action wait(double time) {
        return new Action() {
            private double startTime = -1;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double t;
                if (startTime < 0) {
                    startTime = com.acmerobotics.roadrunner.Actions.now();
                    t = 0;
                } else {
                    t = com.acmerobotics.roadrunner.Actions.now() - startTime;
                }

                // stop after time has elapsed
                return !(t >= time);
            }
        };
    }

    public Action savePose() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                BlackBoard.put(POSE, drive.localizer.getPose());

                return true;
            }
        };
    }

    /* Finalization */

    public Action build() {
        return new RaceAction(
                new SequentialAction(actions),
                shooter.maintainVelocity(),
                led.updateIndicatorAction(),
                robot.estimatePose(),
                savePose()
        );
    }

    public AutoBuilder run() {
        Actions.runBlocking(build());
        return  this;
    }

    public void stop() {
        robot.stop();
    }
}
