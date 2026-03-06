package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.RobotContainer.DriverInputs;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.opencv.core.Algorithm;

import java.util.function.Supplier;

public class DefaultDrive extends CommandBase {
    private final Drive drive;
    private final Supplier<DriverInputs> driveSupplier;
    private final Constants.Alliance alliance;

    public DefaultDrive(Drive drive, Supplier<DriverInputs> driveSupplier, Constants.Alliance alliance) {
        this.drive = drive;
        this.driveSupplier = driveSupplier;
        this.alliance = alliance;

        drive.setHeadingLock(false, 0);

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

        drive.getLocalizer().updateHeading(
                drive.getHeadingCorrected(
                        alliance
                )
        );

        drive.updateGoalHeadingError(drive.getPose(), alliance);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setDriveInputs(0, 0, 0);
    }
}