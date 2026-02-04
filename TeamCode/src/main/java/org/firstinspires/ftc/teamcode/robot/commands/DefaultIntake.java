package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.RobotContainer.DriverInputs;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import java.util.function.Supplier;

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