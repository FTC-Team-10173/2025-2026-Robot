package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.robot.autos.AutoBuilder;

public final class ShooterMath {
    static double getShooterPower(Pose2d pose, AutoBuilder.Alliance alliance) {
        Translation2d translationalPose = new Translation2d(
                pose.position.x,
                pose.position.y
        );

        Translation2d goal = Constants.GoalPoses.get(alliance);

        double distance = translationalPose.getDistance(goal);

        double extra = pose.position.x > 24 ? 0.015 : 0;

        if (distance > 0) {
            return (Constants.Shooter.SLOPE * distance) + Constants.Shooter.INTERCEPT + extra;
        }
        return 0.42;
    }

    static double getGoalError(Pose2d pose, AutoBuilder.Alliance alliance) {
        Translation2d goalPose = Constants.GoalPoses.get(alliance);

        double xDiff = goalPose.getX() - pose.position.x;
        double yDiff = goalPose.getY() - pose.position.y;

        double targetHeading = Math.atan2(yDiff, xDiff);

        double error = targetHeading - pose.heading.toDouble();

        error = Math.atan2(Math.sin(error), Math.cos(error));

        return error;
    }
}
