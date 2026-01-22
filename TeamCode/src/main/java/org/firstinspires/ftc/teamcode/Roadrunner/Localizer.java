package org.firstinspires.ftc.teamcode.Roadrunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.robot.subsystems.Limelight;

/**
 * Interface for localization methods.
 */
public interface Localizer {
    void setPose(Pose2d pose);

    /**
     * Returns the current pose estimate.
     * NOTE: Does not update the pose estimate;
     * you must call update() to update the pose estimate.
     * @return the Localizer's current pose
     */
    Pose2d getPose();

    /**
     * Updates the Localizer's pose estimate.
     * @return the Localizer's current velocity estimate
     */
    PoseVelocity2d update();

    default void estimateLL(Limelight.Botpose botpose) {
        Pose2d currentPose = getPose();

        Translation2d newPose = new Translation2d(
                (botpose.x * 0.1 * 39.37) + (currentPose.position.x * 0.9),
                (botpose.y * 0.1 * 39.37) + (currentPose.position.y * 0.9)
        );

        setPose(new Pose2d(
                newPose.getX(),
                newPose.getY(),
                currentPose.heading.toDouble()
        ));
    }
}
