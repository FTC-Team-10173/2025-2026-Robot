package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;

public class OpenGateCommand extends CommandBase {
    private final Shooter shooter;
    private final Intake intake;

    public OpenGateCommand(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (shooter.isReady(0.5)) {
            intake.openGate();
        }
    }
}