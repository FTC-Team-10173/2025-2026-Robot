package org.firstinspires.ftc.teamcode.robot.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.ShooterMath;
import org.firstinspires.ftc.teamcode.robot.commands.*;
import org.firstinspires.ftc.teamcode.robot.subsystems.*;

public class Robot {
    private final HardwareMap hardwareMap;

    // Subsystems
    private final Drive drive;
    private final Shooter shooter;
    private final Turret turret;
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
        turret = new Turret(hardwareMap);

        limelight.setPipeline(0);
    }

    public Action setPower(Pose2d robotPose) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double power = ShooterMath.getShooterPower(robotPose, alliance);
                shooter.setPower(power);

                return false;
            }
        };
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
    public Turret getTurret() { return turret; }


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