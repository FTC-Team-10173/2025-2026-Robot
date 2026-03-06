package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

/**
 * Utility class for math related to the shooter, turret, and scoring
 */
public final class ShooterMath {
    /**
     *
     * @param pose The robot's current pose
     * @param alliance The robot's alliance (Blue or Red)
     * @return The power (from 0 to 1) that the shooter needs to spin up to
     */
    public static double getShooterPower(Pose2d pose, Constants.Alliance alliance) {
        Translation2d translationalPose = new Translation2d(
                pose.position.x,
                pose.position.y
        );

        Translation2d goal = Constants.GoalPoses.get(alliance);

        double distance = translationalPose.getDistance(goal);

        double powerOffset = pose.position.x > 24 ? -0.04 : 0;

        if (distance > 0) {
            return Constants.Shooter.INTERCEPT * Math.pow(Constants.Shooter.BASE, distance) + powerOffset;
        }

        return 0.45;
    }

    /**
     *
     * @param pose The robot's current pose
     * @param alliance The robot's alliance (Blue or Red)
     * @return The error (in radians) the robot is from the goal
     */
    public static double getGoalError(Pose2d pose, Constants.Alliance alliance) {
        Translation2d goalPose = Constants.GoalPoses.getTurretGoal(alliance);

        double xDiff = goalPose.getX() - pose.position.x;
        double yDiff = goalPose.getY() - pose.position.y;

        double targetHeading = Math.atan2(yDiff, xDiff);

        double error = targetHeading - pose.heading.toDouble();

        return Math.atan2(Math.sin(error), Math.cos(error));
    }

    /**
     * The servos' ranges need to be configured in Constants.java
     *
     * @param pose The robot's current pose
     * @param alliance The robot's alliance (Blue or Red)
     * @return The angle (in degrees) the servos need to turn to
     */
    public static double getTurretTarget(Pose2d pose, Constants.Alliance alliance) {
        double errorRadians = getGoalError(pose, alliance);

        double turretDegrees = Math.toDegrees(errorRadians);

        // Clamp turret to physical turn radius -90 to +90
        return Math.max(-Constants.Turret.RANGE, Math.min(Constants.Turret.RANGE, turretDegrees));
    }
}
