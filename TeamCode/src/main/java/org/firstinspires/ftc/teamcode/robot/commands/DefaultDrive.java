package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.RobotContainer.DriverInputs;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import java.util.function.Supplier;

public class DefaultDrive extends CommandBase {
    private final Drive drive;
    private final Supplier<DriverInputs> driveSupplier;

    public DefaultDrive(Drive drive, Supplier<DriverInputs> driveSupplier) {
        this.drive = drive;
        this.driveSupplier = driveSupplier;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        DriverInputs driveInputs = driveSupplier.get();

        drive.setDriveInputs(
                driveInputs.LeftY,
                driveInputs.LeftX,
                driveInputs.RightX
        );
    }

    @Override
    public void end(boolean interrupted) {
        drive.setDriveInputs(0, 0, 0);
    }
}