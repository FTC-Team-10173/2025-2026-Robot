package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.DriverControls;

import org.firstinspires.ftc.teamcode.robot.subsystems.Limelight.Results;

public class Drive implements Subsystem {

    MecanumDrive drive;
    DriverControls controls;
    Limelight limelight;
    IMU imu;
    double lock_turn;
    PIDController pid;

    // TeleOp constructor
    public Drive(HardwareMap hardwareMap, DriverControls controls, Limelight limelight, IMU imu) {
        // initialize drive with starting pose at origin
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // store driver controls
        this.controls = controls;

        // store limelight
        this.limelight = limelight;

        // store imu
        this.imu = imu;

        // initialize pid controller for heading lock
        pid = new PIDController(
                Constants.Drive.HEADING_KP,
                Constants.Drive.HEADING_KI,
                Constants.Drive.HEADING_KD
        );
    }

    // Autonomous constructor
    public Drive(HardwareMap hardwareMap, Limelight limelight, IMU imu) {
        // initialize drive with starting pose at origin
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // store limelight
        this.limelight = limelight;

        // store imu
        this.imu = imu;

        // initialize pid controller for heading lock
        pid = new PIDController(
                Constants.Drive.HEADING_KP,
                Constants.Drive.HEADING_KI,
                Constants.Drive.HEADING_KD
        );
    }

    // periodic method to be called in main loop
    public void periodic() {
        // reset imu yaw
        if (controls.imuResetPressed()) {
            resetYaw();
        }

        // update heading lock
        if (controls.lockDrivePressed()) {
            Results results = limelight.results;
            if (results.hasTarget) { // if the limelight sees a tag
                lock(limelight.results.tx);
            }
        }

        // set drive powers based on driver controls
        setDrivePowers(
                controls.driver,
                controls.lockDrivePressed()
        );

        // update drive pose estimate
        drive.updatePoseEstimate();
    }

    public boolean isHealthy() {
        return drive != null && imu != null;
    }

    public void stop() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    public void updateTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        Pose2d pose = getPose();
        telemetry.addLine();
        telemetry.addData(getName() + " X (Inches)", "%.2f", pose.position.x);
        telemetry.addData(getName() + " Y (Inches)", "%.2f", pose.position.y);
        telemetry.addData(getName() + " Heading (Degrees)", "%.2f", Math.toDegrees(pose.heading.real));
        telemetry.addData(getName() + " IMU Yaw (Degrees)", "%.2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData(getName() + " Healthy", isHealthy());
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
    }

    // lock heading using pid controller
    public void lock(double error) {
        lock_turn = -pid.calculate(error);
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
