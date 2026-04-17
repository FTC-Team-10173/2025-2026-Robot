package org.firstinspires.ftc.teamcode.robot.autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.autonomous.paths.AutonomousPath;

import com.pedropathing.paths.Path;

import java.util.List;
import java.util.function.Function;

@Configurable
public class AutoBase extends OpMode {

    private TelemetryManager panelsTelemetry;
    private final Pose startPose;
    private final Function<Follower, AutonomousPath> pathFactory;
    private AutonomousPath path;
    private List<PathChain> paths;
    private int pathIndex = 0;
    private Follower follower;
    private int pathState;

    public AutoBase(Pose startPose, Function<Follower, AutonomousPath> pathFactory) {
        this.startPose = startPose;
        this.pathFactory = pathFactory;
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        path = pathFactory.apply(follower);

        paths = path.getPaths();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Route", path.toString());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        nextPath();

        follower.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public void nextPath() {
        if (!follower.isBusy()) {
            follower.followPath(paths.get(pathIndex));
            pathIndex++;
        }
    }

    private int autonomousPathUpdate() {
        // state machine here
        return 0;
    }
}