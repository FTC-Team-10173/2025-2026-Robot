package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeCommand extends CommandBase {
    private final Intake intake;
    private final DoubleSupplier powerSupplier;
    private final BooleanSupplier isFullIntakeSupplier;

    public IntakeCommand(Intake intake, DoubleSupplier powerSupplier, BooleanSupplier isFullIntake) {
        this.intake = intake;
        this.powerSupplier = powerSupplier;
        this.isFullIntakeSupplier = isFullIntake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Initial setup based on mode
        double power = powerSupplier.getAsDouble();

        if (power < 0) {
            // Outtake mode
            intake.outtake();
        } else if (isFullIntakeSupplier.getAsBoolean()) {
            // Full intake mode
            intake.fullIntake();
        } else {
            // Half intake mode
            intake.halfIntake();
        }
    }

    @Override
    public void execute() {
        // Continue intake based on current mode
        double power = powerSupplier.getAsDouble();

        if (power < 0) {
            intake.outtake();
        } else if (isFullIntakeSupplier.getAsBoolean()) {
            intake.fullIntake();
        } else {
            intake.halfIntake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }
}