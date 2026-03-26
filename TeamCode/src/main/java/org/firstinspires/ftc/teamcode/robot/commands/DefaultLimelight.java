package org.firstinspires.ftc.teamcode.robot.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;

import org.firstinspires.ftc.teamcode.Roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.robot.subsystems.Limelight;

import java.util.function.DoubleSupplier;

public class DefaultLimelight extends CommandBase {
    private final Limelight limelight;
    private final Localizer poseEstimator;
    private final DoubleSupplier headingSupplier;
    private final DoubleSupplier turretSupplier;

    public DefaultLimelight(Limelight limelight, Localizer poseEstimator, DoubleSupplier headingSupplier, DoubleSupplier turretSupplier) {
        this.limelight = limelight;
        this.poseEstimator =  poseEstimator;
        this.headingSupplier = headingSupplier;
        this.turretSupplier = turretSupplier;

        addRequirements(limelight);
    }

    @Override
    public void execute() {
         LLResult result = limelight.getResults();

        limelight.updateHeading(headingSupplier.getAsDouble(), turretSupplier.getAsDouble());


        if (result != null && result.isValid()) {
             poseEstimator.addLimelight(result);
         }
    }
}