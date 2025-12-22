package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

@TeleOp(name="Robot", group="2025-2026")
public class Main extends LinearOpMode {
    RobotContainer container;

    @Override
    public void runOpMode() throws InterruptedException {
        container = new RobotContainer(hardwareMap, new GamepadEx(gamepad1));
        waitForStart();

        while (opModeIsActive()) {
            container.periodic(telemetry);
            telemetry.update();
        }
    }
}
