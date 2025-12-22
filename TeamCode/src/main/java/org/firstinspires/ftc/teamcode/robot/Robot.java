package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.LED;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.VisionController;
import org.firstinspires.ftc.teamcode.robot.RobotState;

public class Robot {
    public final Drive drive;
    public final Shooter shooter;
    public final Intake intake;
    public final LED led;
    public final VisionController vision;
    public RobotState robotState;

    // TeleOp constructor
    public Robot(HardwareMap hardwareMap, DriverControls controls) {
        robotState = new RobotState(controls);
        led = new LED(hardwareMap, robotState);
        vision = new VisionController(hardwareMap);
        shooter = new Shooter(hardwareMap, controls, robotState, vision);
        intake = new Intake(hardwareMap, controls);
        drive = new Drive(hardwareMap, controls, vision);
    }

    // Autonomous constructor
    public Robot(HardwareMap hardwareMap) {
        robotState = new RobotState();
        led = new LED(hardwareMap, robotState);
        vision = new VisionController(hardwareMap);
        shooter = new Shooter(hardwareMap, robotState, vision);
        intake = new Intake(hardwareMap);
        drive = new Drive(hardwareMap, vision);
    }
}
