package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

public class DefaultIntake extends CommandBase {
    private final Intake intake;

    public DefaultIntake(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.stopIntake();
    }
}