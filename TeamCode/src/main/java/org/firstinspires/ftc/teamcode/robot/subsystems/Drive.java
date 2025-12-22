package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.DriverControls;

public class Drive {

    MecanumDrive drive;
    DriverControls controls;
    VisionController vision;
    IMU imu;
    double lock_turn;
    PIDController pid;

    // TeleOp constructor
    public Drive(HardwareMap hardwareMap, DriverControls controls, VisionController vision) {
        // initialize drive with starting pose at origin
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // store driver controls
        this.controls = controls;

        // store vision controller
        this.vision = vision;

        // initialize imu
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);

        // initialize pid controller for heading lock
        pid = new PIDController(
            0.02, 0.0, 0.0
        );
    }

    // Autonomous constructor
    public Drive(HardwareMap hardwareMap, VisionController vision) {
        // initialize drive with starting pose at origin
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // store vision controller
        this.vision = vision;

        // initialize imu
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);

        // initialize pid controller for heading lock
        pid = new PIDController(
                0.02, 0.0, 0.0
        );
    }

    // periodic method to be called in main loop
    public void periodic() {
        // reset yaw if requested
        if (controls.resetYawPressed()) {
            resetYaw();
        }

        // update heading lock
        lock(vision.bearing);

        // set drive powers based on driver controls
        setDrivePowers(
                controls.driver,
                controls.lockDrivePressed()
        );

        // update drive pose estimate
        drive.updatePoseEstimate();
    }

    // set drive powers based on field-centric controls
    public void setDrivePowers(GamepadEx driver, boolean lock) {
        // get robot-centric input from gamepad
        Vector2d input = new Vector2d(
            driver.getLeftY(),
            -driver.getLeftX()
        );

        // get robot heading from imu
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // convert to field-centric input
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        // rotate input vector by -heading
        double fieldX = input.x * cos + input.y * sin;
        double fieldY = input.y * cos - input.x * sin;

        // create field-centric vector
        Vector2d fieldCentricInput = new Vector2d(fieldX, fieldY);

        // set drive powers
        if(lock) { // heading lock
            drive.setDrivePowers(new PoseVelocity2d(
                    fieldCentricInput,
                    lock_turn // use pid output for turn power
            ));
        } else { // normal driving
            drive.setDrivePowers(new PoseVelocity2d(
                    fieldCentricInput,
                    -driver.getRightX()
            ));
        }

        drive.updatePoseEstimate(); // update pose estimate
    }

    // lock heading using pid controller
    public void lock(double bearing) {
        lock_turn = -pid.calculate(bearing);
    }

    // get current pose
    public Pose2d getPose() {
        return drive.localizer.getPose();
    }

    // reset yaw to zero
    public void resetYaw() {
        imu.resetYaw();
    }
}
