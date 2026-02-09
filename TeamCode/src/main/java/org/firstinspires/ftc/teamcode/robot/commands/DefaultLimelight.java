package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.Roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.robot.subsystems.Limelight;

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
         LLResult result = limelight.getResults();

         if (result != null && result.isValid()) {
             poseEstimator.addLimelight(result);
         }
    }
}