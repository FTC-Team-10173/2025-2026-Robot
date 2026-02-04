package org.firstinspires.ftc.teamcode.Roadrunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.robot.autos.AutoBuilder;
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

    Translation2d LLpose = new Translation2d();

    default void addLimelight(LLResult result) {

    };
}
