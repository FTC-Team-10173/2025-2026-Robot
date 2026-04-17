package org.firstinspires.ftc.teamcode.robot.autonomous.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import java.util.ArrayList;
import java.util.List;

public class BlueCloseFull implements AutonomousPath {
    public List<PathChain> paths = new ArrayList<>();

    public BlueCloseFull(Follower follower) {
        paths.add(follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(25.000, 127.000),
                                new Pose(32.000, 120.000),
                                new Pose(48.000, 96.000),
                                new Pose(60.000, 84.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build());

        paths.add(follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60.000, 84.000),
                                new Pose(36.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(36.000, 84.000),
                                new Pose(14.000, 84.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build());

        paths.add(follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(14.000, 84.000),
                                new Pose(36.000, 84.000),
                                new Pose(48.000, 96.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build());

        paths.add(follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(48.000, 96.000),
                                new Pose(48.000, 60.000),
                                new Pose(36.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(36.000, 60.000),
                                new Pose(14.000, 60.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build());

        paths.add(follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(14.000, 60.000),
                                new Pose(36.000, 60.000),
                                new Pose(60.000, 84.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build());

        paths.add(follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(48.000, 36.000),
                                new Pose(36.000, 36.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(36.000, 36.000),
                                new Pose(14.000, 36.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build());

        paths.add(follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(14.000, 36.000),
                                new Pose(36.000, 36.000),
                                new Pose(48.000, 72.000),
                                new Pose(60.000, 84.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build());

        paths.add(follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60.000, 84.000),
                                new Pose(24.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(180))
                .build());
    }

    @Override
    public List<PathChain> getPaths() {
        return paths;
    }
}