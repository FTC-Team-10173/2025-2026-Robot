package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.RobotContainer.DriverInputs;
import org.firstinspires.ftc.teamcode.robot.autos.AutoBuilder;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.Limelight;

import java.util.function.BiFunction;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class HeadingLockCommand extends CommandBase {
    private final Drive drive;
    private final AutoBuilder.Alliance alliance;
    private final Supplier<Pose2d> poseSupplier;
    private final BiFunction<Pose2d, AutoBuilder.Alliance, Double> errorSupplier;
    private final Supplier<DriverInputs> driveSupplier;

    public HeadingLockCommand(Drive drive, AutoBuilder.Alliance alliance, Supplier<Pose2d> poseSupplier, BiFunction<Pose2d, AutoBuilder.Alliance, Double> errorSupplier, Supplier<DriverInputs> driveSupplier) {
        this.drive = drive;
        this.alliance = alliance;
        this.poseSupplier = poseSupplier;
        this.errorSupplier =errorSupplier;
        this.driveSupplier = driveSupplier;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.setHeadingLock(true, errorSupplier.apply(poseSupplier.get(), alliance));

        DriverInputs driveInputs = driveSupplier.get();

        drive.setDriveInputs(
                driveInputs.LeftY,
                driveInputs.LeftX,
                driveInputs.RightX
        );
    }

    @Override
    public void end(boolean interrupted) {
        drive.setHeadingLock(false, 0);
    }
}