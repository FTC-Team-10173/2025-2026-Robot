package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.autos.AutoBuilder.Alliance;
import org.firstinspires.ftc.teamcode.robot.commands.*;
import org.firstinspires.ftc.teamcode.robot.subsystems.*;

public class RobotContainer {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Logger logger;

    // Subsystems
    private final Drive drive;
    private final Shooter shooter;
    private final Intake intake;
    private final LED led;
    private final Limelight limelight;

    // Controls
    private final DriverControls controls;

    // Alliance
    private final Alliance alliance;

    public RobotContainer(HardwareMap hardwareMap, GamepadEx driverGamepad, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.logger = new Logger("TeleOp_" + System.currentTimeMillis());

        // Initialize subsystems
        limelight = new Limelight(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        led = new LED(hardwareMap);
        drive = new Drive(hardwareMap);

        limelight.setPipeline(0);

        // Initialize controls
        controls = new DriverControls(driverGamepad);

        // Initialize alliance
        this.alliance =Constants.BlackBoard.containsKey(Constants.Keys.ALLIANCE) ?
                Alliance.BLUE :
                (Alliance) Constants.BlackBoard.get(Constants.Keys.ALLIANCE);

        // Configure button bindings
        configureBindings();

        // Register default commands
        registerDefaultCommands();
    }

    private void configureBindings() {
        // Shoot command - Left Bumper
        controls.shootTrigger.whileActiveOnce(
                new ShootCommand(shooter, this::calculateShooterPower)
        );

        // Full Intake - Right Bumper
        controls.fullIntakeTrigger.whileActiveOnce(
                new IntakeCommand(intake, () -> 1.0, () -> true)
        );

        // Half Intake - Right Trigger
        controls.intakeTrigger.whileActiveOnce(
                new IntakeCommand(intake, () -> 1.0, () -> false)
        );

        // Outtake - Left Trigger
        controls.outtakeTrigger.whileActiveOnce(
                new IntakeCommand(intake, () -> -1.0, () -> false)
        );

        // Yaw Reset - Back button
        controls.yawResetTrigger.whenActive(
                new ResetYawCommand(drive)
        );

        // Heading Lock - A button
        controls.lockDriveTrigger.whileActiveOnce(
                new HeadingLockCommand(drive, limelight, this::getDriveInputs)
        );
    }

    private void registerDefaultCommands() {
        // Set default drive command
        drive.setDefaultCommand(
                new DefaultDrive(
                        drive,
                        this::getDriveInputs
                )
        );
    }

    private DriverInputs getDriveInputs() {
        return new DriverInputs(
                controls.driver.getLeftY(),
                -controls.driver.getLeftX(),
                controls.driver.getRightX()
        );
    }

    private double calculateShooterPower() {
        // Calculate based on distance to target
        double distance = limelight.getTargetDistance();
        if (distance > 0) {
            return (Constants.Shooter.SLOPE * distance) + Constants.Shooter.INTERCEPT;
        }
        return Constants.ShootingPower.CLOSE; // Default
    }

    public void periodic() {
        // Read gamepad inputs
        controls.readButtons();

        // Update robot state based on subsystem status
        updateRobotState();

        // Run the FTCLib command scheduler
        CommandScheduler.getInstance().run();

        // Update telemetry periodically
        updateTelemetry();
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

    private void updateTelemetry() {
        drive.updateTelemetry(telemetry, logger);
        shooter.updateTelemetry(telemetry, logger);
        intake.updateTelemetry(telemetry, logger);
        led.updateTelemetry(telemetry, logger);
        limelight.updateTelemetry(telemetry, logger);

        telemetry.update();
    }

    public void stop() {
        CommandScheduler.getInstance().reset();

        drive.stop();
        shooter.stop();
        intake.stop();
        led.stop();
        limelight.stop();

        logger.save();
    }

    // Getters for subsystems
    public Drive getDrive() { return drive; }
    public Shooter getShooter() { return shooter; }
    public Intake getIntake() { return intake; }
    public LED getLed() { return led; }
    public Limelight getLimelight() { return limelight; }

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