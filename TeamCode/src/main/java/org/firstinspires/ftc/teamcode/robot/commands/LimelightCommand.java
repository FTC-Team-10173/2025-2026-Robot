package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.subsystems.Limelight;

import java.util.function.IntSupplier;

public class LimelightCommand extends CommandBase {
    private final Limelight limelight;
    private final IntSupplier pipelineSupplier;

    public LimelightCommand(Limelight limelight, IntSupplier pipelineSupplier) {
        this.limelight = limelight;
        this.pipelineSupplier = pipelineSupplier;

        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        // Set initial pipeline if needed
        int pipeline = pipelineSupplier.getAsInt();
        if (pipeline >= 0) {
            limelight.setPipeline(pipeline);
        }
    }

    @Override
    public void execute() {
        // Update pipeline if changed
        int pipeline = pipelineSupplier.getAsInt();
        if (pipeline >= 0 && pipeline != limelight.getPipeline()) {
            limelight.setPipeline(pipeline);
        }
    }
}