package org.firstinspires.ftc.teamcode.robot.autonomous.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import java.util.ArrayList;
import java.util.List;

public class RedCloseFull implements AutonomousPath {
    private final List<PathChain> paths = new ArrayList<>();

    public RedCloseFull(Follower follower) {
        paths.add(follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(119.000, 127.000),
                                new Pose(112.000, 120.000),
                                new Pose(96.000, 96.000),
                                new Pose(84.000, 84.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build());

        paths.add(follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(84.000, 84.000),
                                new Pose(108.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(108.000, 84.000),
                                new Pose(130.000, 84.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build());

        paths.add(follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(130.000, 84.000),
                                new Pose(108.000, 84.000),
                                new Pose(96.000, 96.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build());

        paths.add(follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(96.000, 96.000),
                                new Pose(96.000, 60.000),
                                new Pose(108.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(108.000, 60.000),
                                new Pose(130.000, 60.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build());

        paths.add(follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(130.000, 60.000),
                                new Pose(108.000, 60.000),
                                new Pose(84.000, 84.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build());

        paths.add(follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(84.000, 84.000),
                                new Pose(96.000, 36.000),
                                new Pose(108.000, 36.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(108.000, 36.000),
                                new Pose(130.000, 36.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build());

        paths.add(follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(130.000, 36.000),
                                new Pose(108.000, 36.000),
                                new Pose(96.000, 72.000),
                                new Pose(84.000, 84.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build());

        paths.add(follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(84.000, 84.000),
                                new Pose(120.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build());
    }

    @Override
    public List<PathChain> getPaths() {
        return paths;
    }
}