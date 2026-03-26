package org.firstinspires.ftc.teamcode.robot.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.Gate;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

public class OuttakeCommand extends CommandBase {
    private final Intake intake;
    private final Gate gate;

    public OuttakeCommand(Intake intake, Gate gate) {
        this.intake = intake;
        this.gate = gate;

        addRequirements(intake, gate);
    }

    @Override
    public void execute() {
        intake.outtake();
        gate.open();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }
}