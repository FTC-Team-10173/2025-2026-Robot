package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.Limelight;

public class HeadingLockCommand extends CommandBase {
    private final Drive drive;
    private final Limelight limelight;
    private final RobotState robotState;

    public HeadingLockCommand(Drive drive, Limelight limelight, RobotState robotState) {
        this.drive = drive;
        this.limelight = limelight;
        this.robotState = robotState;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        robotState.set(RobotState.State.ALIGNING);
    }

    @Override
    public void execute() {
        Limelight.Results results = limelight.getResults();

        if (results.hasTarget) {
            // Use tx (horizontal angle error) to align
            double targetError = results.tx;
            drive.setHeadingLock(true, targetError);
            robotState.set(RobotState.State.ALIGNING);
        } else {
            // No target, disable heading lock
            drive.setHeadingLock(false, 0);
            robotState.set(RobotState.State.IDLE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.setHeadingLock(false, 0);
        robotState.set(RobotState.State.IDLE);
    }
}