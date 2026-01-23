package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.Roadrunner.OTOSLocalizer;
import org.firstinspires.ftc.teamcode.Roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.robot.autos.AutoBuilder;

public class PoseEstimator implements Localizer {

    private final ThreeDeadWheelLocalizer tdwLocalizer;
    private final OTOSLocalizer otosLocalizer;

    public PoseEstimator(HardwareMap hardwareMap, double inPerTick, Pose2d pose) {
        tdwLocalizer = new ThreeDeadWheelLocalizer(hardwareMap, inPerTick, pose);
        otosLocalizer = new OTOSLocalizer(hardwareMap, pose);
    }

    @Override
    public void setPose(Pose2d pose) {
        tdwLocalizer.setPose(pose);
        otosLocalizer.setPose(pose);
    }

    @Override
    public Pose2d getPose() {
        Pose2d tdwPose = tdwLocalizer.getPose();
        Pose2d otosPose = otosLocalizer.getPose();

        // Simple average of the two poses
        double x = (tdwPose.position.x + otosPose.position.x) / 2.0;
        double y = (tdwPose.position.y + otosPose.position.y) / 2.0;
        double heading = (tdwPose.heading.toDouble() + otosPose.heading.toDouble()) / 2.0;

        return new Pose2d(x, y, heading);
    }

    @Override
    public PoseVelocity2d update() {
        PoseVelocity2d tdwVelPose = tdwLocalizer.update();
        PoseVelocity2d otosVelPose = otosLocalizer.update();

        // Simple average of the two poses
        double x = (tdwVelPose.linearVel.x + tdwVelPose.linearVel.x) / 2.0;
        double y = (tdwVelPose.linearVel.y + tdwVelPose.linearVel.y) / 2.0;
        double angVel = (otosVelPose.angVel + otosVelPose.angVel) / 2.0;

        return new PoseVelocity2d(
                new Vector2d(x, y),
                angVel
        );
    }

    public double getGoalDistance(AutoBuilder.Alliance alliance) {
        return 0.0;
    }
}
