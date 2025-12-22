package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.robot.subsystems.LED;
import org.firstinspires.ftc.teamcode.robot.subsystems.VisionController;

import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

@TeleOp(name="Controller", group="2025-2026")
public class Controller extends LinearOpMode {
    Drive mecanumDrive;
    Shooter shooter;
    Intake intake;
    LED led;

    GamepadEx driver;

    VisionController visionController;

    int GOAL_ID = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initiating shooter and intake subsystem classes
        led = new LED(hardwareMap);
        shooter = new Shooter(hardwareMap, led);
        intake = new Intake(hardwareMap);

        // Initiating drive train subsystem class
        mecanumDrive = new Drive(hardwareMap);

        // Initiating driver gamepad class
        driver = new GamepadEx(gamepad1);

        // Initiating vision class
        visionController = new VisionController(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            visionController.setManualExposure(); // Setting the exposure and gain for the camera

            if(driver.getButton(GamepadKeys.Button.BACK)) { // Reset yaw when back button pressed
                mecanumDrive.resetYaw();
            }

            // Feeding drive train the driver gamepad inputs or locking to goal based on bearing
            if(driver.getButton(GamepadKeys.Button.A)) {
                mecanumDrive.lock(visionController.getBearing(GOAL_ID));
                mecanumDrive.setDrivePowers(driver, true); // Giving drive train driver gamepad
            } else {
                mecanumDrive.setDrivePowers(driver, false); // Giving drive train driver gamepad
            }

            /*
             * right trigger for half intake
             * left trigger for outtake
             * right bumper for full intake
             * B button for half outtake
             */
            intake.setPower(
                    driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                    driver.getButton(GamepadKeys.Button.RIGHT_BUMPER), driver.getButton(GamepadKeys.Button.B)
            );

            // Setting shooter power based on distance from goal
            shooter.setPower(
                    visionController.getDistance(GOAL_ID)
            );

            // Telling shooter to spin using previously set power
            // Left bumper to spin shooter
            shooter.spin(
                    driver.getButton(GamepadKeys.Button.LEFT_BUMPER)
            );

            // Updating LED status
            led.update();

            // Displaying data for reference
            telemetry.addData("GOAL_ID", GOAL_ID);
            telemetry.addData("bearing", visionController.getBearing(GOAL_ID));
            telemetry.addData("distance", visionController.getDistance(GOAL_ID));
            telemetry.addData("power", shooter.power);
            telemetry.addData("velocity", shooter.flywheel.getVelocity());

            // Update telemetry
            telemetry.update();
        }
    }
}