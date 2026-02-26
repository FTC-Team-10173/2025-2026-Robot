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
    static double getShooterPower(Pose2d pose, Constants.Alliance alliance) {
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

    /**
     *
     * @param pose The robot's current pose
     * @param alliance The robot's alliance (Blue or Red)
     * @return The error (in radians) the robot is from the goal
     */
    static double getGoalError(Pose2d pose, Constants.Alliance alliance) {
        Translation2d goalPose = Constants.GoalPoses.get(alliance);

        double xDiff = goalPose.getX() - pose.position.x;
        double yDiff = goalPose.getY() - pose.position.y;

        double targetHeading = Math.atan2(yDiff, xDiff);

        double error = targetHeading - pose.heading.toDouble();

        error = Math.atan2(Math.sin(error), Math.cos(error));

        return error;
    }

    /**
     * The servos' ranges need to be configured in Constants.java
     *
     * @param pose The robot's current pose
     * @param alliance The robot's alliance (Blue or Red)
     * @return The angle (in degrees) the servos need to turn to
     */
    static double getTurretTarget(Pose2d pose, Constants.Alliance alliance) {
        double errorRadians = getGoalError(pose, alliance);

        double turretDegrees = Math.toDegrees(errorRadians);

        // Clamp turret to physical turn radius -90 to +90
        turretDegrees = Math.max(-Constants.Turret.RANGE, Math.min(Constants.Turret.RANGE, turretDegrees));

        // Convert turret degrees to servo degrees using gear ratio
        double servoDegrees = turretDegrees / Constants.Turret.GEAR_RATIO;

        // Centered at 900
        return (Constants.Turret.RANGE_MAX_ANGLE / 2.0) + servoDegrees;
    }
}
