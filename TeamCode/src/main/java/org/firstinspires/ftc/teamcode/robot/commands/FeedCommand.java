package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.Gate;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

import java.util.function.Supplier;

public class FeedCommand extends CommandBase {
    private final Intake intake;
    private final Gate gate;
    private final Supplier<Pose2d> poseSupplier;

    public FeedCommand(Intake intake, Gate gate, Supplier<Pose2d> poseSupplier) {
        this.intake = intake;
        this.gate = gate;
        this.poseSupplier = poseSupplier;

        addRequirements(intake, gate);
    }

    @Override
    public void execute() {
        Pose2d pose = poseSupplier.get();

        if (pose.position.x > 24) {
            intake.farFeed();
        }
        else {
            intake.intake();
        }

        gate.open();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }
}