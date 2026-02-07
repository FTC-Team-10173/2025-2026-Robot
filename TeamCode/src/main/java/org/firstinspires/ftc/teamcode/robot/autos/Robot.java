package org.firstinspires.ftc.teamcode.robot.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.autos.AutoBuilder.Alliance;
import org.firstinspires.ftc.teamcode.robot.commands.*;
import org.firstinspires.ftc.teamcode.robot.subsystems.*;

public class Robot {
    private final HardwareMap hardwareMap;

    // Subsystems
    private final Drive drive;
    private final Shooter shooter;
    private final Intake intake;
    private final LED led;
    private final Limelight limelight;

    // BlackBoard
    private final Constants.Alliance alliance;
    private final Pose2d startPose;

    public Robot(HardwareMap hardwareMap, Constants.Alliance alliance, Pose2d startPose) {
        this.hardwareMap = hardwareMap;

        this.alliance = alliance;
        Constants.BlackBoard.put(Constants.Keys.ALLIANCE, alliance);

        this.startPose = startPose;
        Constants.BlackBoard.put(Constants.Keys.POSE, startPose);

        // Initialize subsystems
        limelight = new Limelight(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        led = new LED(hardwareMap);
        drive = new Drive(hardwareMap);

        limelight.setPipeline(0);
    }

    public Action setPower(Pose2d robotPose) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double power = calculateShooterPower(robotPose, alliance);
                shooter.setPower(power);

                return false;
            }
        };
    }

    private double calculateShooterPower(Pose2d robotPose, Constants.Alliance alliance) {
        Translation2d GoalPose = Constants.GoalPoses.get(alliance);

        double distance = Math.hypot(
                GoalPose.getX() - robotPose.position.x,
                GoalPose.getY() - robotPose.position.y
        );

        if (distance > 0) {
            return (Constants.Shooter.SLOPE * distance) + Constants.Shooter.INTERCEPT;
        }
        return Constants.ShootingPower.CLOSE; // Default
    }

    private void updateRobotState() {
        if (shooter.isRunning()) {
            if (shooter.isReady()) {
                led.set(LED.State.SHOOTING_READY);
            } else {
                led.set(LED.State.SPINNING_UP);
            }
        } else {
            led.set(LED.State.IDLE);
        }
    }

    public void stop() {
        CommandScheduler.getInstance().reset();

        drive.stop();
        shooter.stop();
        intake.stop();
        led.stop();
    }

    // Getters for subsystems
    public Drive getDrive() { return drive; }
    public Shooter getShooter() { return shooter; }
    public Intake getIntake() { return intake; }
    public LED getLed() { return led; }
    public Limelight getLimelight() { return limelight; }

    public Action estimatePose() {
        return new Action() {
            final Localizer poseEstimator = drive.getLocalizer();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                LLResult result = limelight.getResults();

                if (result != null && result.isValid()) {
                    poseEstimator.addLimelight(result);
                }

                return true;
            }
        };
    }

    public static class DriverInputs {
        public double LeftY;
        public double LeftX;
        public double RightX;
        public DriverInputs(double LeftY, double LeftX, double RightX) {
            this.LeftY = LeftY;
            this.LeftX = LeftX;
            this.RightX = RightX;
        }
    }

}