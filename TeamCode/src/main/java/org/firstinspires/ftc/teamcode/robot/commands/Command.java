package org.firstinspires.ftc.teamcode.robot.commands;

import org.firstinspires.ftc.teamcode.robot.subsystems.Subsystem;

import java.util.List;

public interface Command {
    void initialize();
    void execute();
    boolean isFinished();
    void end(boolean interrupted);

    List<Subsystem> getRequirements();
}
