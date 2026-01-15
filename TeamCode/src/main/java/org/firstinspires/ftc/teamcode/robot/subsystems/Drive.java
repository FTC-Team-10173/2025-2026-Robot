package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.DriverControls;

import org.firstinspires.ftc.teamcode.robot.subsystems.Limelight.Results;

public class Drive implements Subsystem {

    public MecanumDrive drive;
    DriverControls controls;
    Limelight limelight;
    double lock_turn;
    PIDController pid;
    IMU imu;
    private boolean lastYawReset = false;

    // TeleOp constructor
    public Drive(HardwareMap hardwareMap, DriverControls controls, Limelight limelight, IMU imu) {
        // initialize drive with starting pose at origin
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // store driver controls
        this.controls = controls;

        // store imu
        this.imu = imu;

        // store limelight
        this.limelight = limelight;

        // initialize pid controller for heading lock
        pid = new PIDController(
                Constants.Drive.HEADING_KP,
                Constants.Drive.HEADING_KI,
                Constants.Drive.HEADING_KD
        );
    }

    // Autonomous constructor
    public Drive(HardwareMap hardwareMap, Limelight limelight, IMU imu, Pose2d startPose) {
        // initialize drive with starting pose at origin
        drive = new MecanumDrive(hardwareMap, startPose);

        // store imu
        this.imu = imu;

        // store limelight
        this.limelight = limelight;

        // initialize pid controller for heading lock
        pid = new PIDController(
                Constants.Drive.HEADING_KP,
                Constants.Drive.HEADING_KI,
                Constants.Drive.HEADING_KD
        );
    }

    // periodic method to be called in main loop
    public void periodic() {
        boolean yawReset = controls.yawResetPressed();

        // reset yaw
        if (yawReset && !lastYawReset) {
            resetYaw();
        }
        lastYawReset = yawReset;

        // update heading lock
        if (controls.lockDrivePressed()) {
            Results results = limelight.results;
            if (results.hasTarget) { // if the limelight sees a tag
                lock(-results.tx);
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
        return drive != null;
    }

    public void stop() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    public void updateTelemetry(Telemetry telemetry, TelemetryPacket packet) {
        Pose2d pose = getPose();
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        telemetry.addLine();
        telemetry.addData(getName() + " X (Inches)", "%.2f", pose.position.x);
        telemetry.addData(getName() + " Y (Inches)", "%.2f", pose.position.y);
        telemetry.addData(getName() + " Heading (Degrees)", "%.2f", Math.toDegrees(pose.heading.real));
        telemetry.addData(getName() + " Heading (Radians)", "%.2f", pose.heading.real);
        telemetry.addData(getName() + " IMU (Degrees)", "%.2f", Math.toDegrees(heading));
        telemetry.addData(getName() + " IMU (Radians)", "%.2f", heading);
        telemetry.addData(getName() + " Healthy", isHealthy());
    }

    // set drive powers based on field-centric controls
    public void setDrivePowers(GamepadEx driver, boolean lock) {

        double leftY = Math.abs(driver.getLeftY()) > Constants.Drive.DEADZONE ? driver.getLeftY() : 0;
        double leftX = Math.abs(driver.getLeftX()) > Constants.Drive.DEADZONE ? -driver.getLeftX() : 0;
        double rightX = Math.abs(driver.getRightX()) > Constants.Drive.DEADZONE ? -driver.getRightX() : 0;

        // get robot-centric input from gamepad
        Vector2d input = new Vector2d(
                leftY,
                leftX // This is fine trust
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

    public Action estimatePose() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // get limelight botpose
                Limelight.Botpose botpose = limelight.getBotpose();

                if (botpose != null && botpose.result.isValid() && botpose.result.getStaleness() < 30) {
                    // update localizer pose with estimate
                    drive.localizer.setPose(
                            getPoseEstimation(botpose)
                    );
                }


                return false;
            }
        };
    }

    @NonNull
    private Pose2d getPoseEstimation(Limelight.Botpose botpose) {
        Pose2d currentPose = drive.localizer.getPose();

        Translation2d newPose = new Translation2d(
                (botpose.x * 0.1) + (currentPose.position.x * 0.9),
                (botpose.y * 0.1) + (currentPose.position.y * 0.9)
        );

        return new Pose2d(
                newPose.getX(),
                newPose.getY(),
                currentPose.heading.toDouble()
        );
    }
}
