package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

@TeleOp(name="Robot", group="2025-2026")
public class Main extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotContainer robotContainer = new RobotContainer(hardwareMap, new GamepadEx(gamepad1), telemetry);

        waitForStart();

        while (opModeIsActive()) {
            robotContainer.periodic();
        }

//        robotContainer.getLogger().save();
//        CommandScheduler.getInstance().reset();
    }
}