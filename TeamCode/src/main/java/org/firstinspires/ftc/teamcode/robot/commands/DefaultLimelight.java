package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.robot.PoseEstimator;
import org.firstinspires.ftc.teamcode.robot.RobotContainer.DriverInputs;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Limelight;

import java.util.function.Supplier;

public class DefaultLimelight extends CommandBase {
    private final Limelight limelight;
    private final Localizer poseEstimator;

    public DefaultLimelight(Limelight limelight, Localizer poseEstimator) {
        this.limelight = limelight;
        this.poseEstimator =  poseEstimator;

        addRequirements(limelight);
    }

    @Override
    public void execute() {
         LLResult result = limelight.getResults().result;

         if (result != null && result.isValid()) {
             poseEstimator.addLimelight(result);
         }
    }
}