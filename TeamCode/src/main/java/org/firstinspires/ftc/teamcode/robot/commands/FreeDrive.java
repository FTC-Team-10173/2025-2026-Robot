package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import java.util.function.DoubleSupplier;

public class FreeDrive extends CommandBase {
    private final Drive drive;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotateSupplier;

    public FreeDrive(Drive drive, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotate) {
        this.drive = drive;
        this.forwardSupplier = forward;
        this.strafeSupplier = strafe;
        this.rotateSupplier = rotate;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.setDriveInputs(
                forwardSupplier.getAsDouble(),
                strafeSupplier.getAsDouble(),
                rotateSupplier.getAsDouble()
        );
    }

    @Override
    public void end(boolean interrupted) {
        drive.setDriveInputs(0, 0, 0);
    }
}