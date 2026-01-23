package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;

public class ResetYawCommand extends InstantCommand {
    public ResetYawCommand(Drive drive) {
        super(
                drive::resetYaw
        );
    }
}