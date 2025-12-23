package org.firstinspires.ftc.teamcode.robot.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Vision implements Subsystem {

    public double distance, bearing;

    AprilTagProcessor aprilTagProcessor;
    VisionPortal visionPortal;
    int exposure, gain;

    // constructor
    public Vision(HardwareMap hardwareMap) {
        // initialize webcam
        WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        // initialize april tag processor
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        // initialize vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .addProcessors(aprilTagProcessor)
                .build();

        // set default exposure and gain
        exposure = 100;
        gain = 1;
    }

    /**
     * Periodic method to be called in main loop
     */
    public void periodic() {
        // set manual exposure and gain
        setManualExposure();

        // get distance and bearing to tag
        distance = getDistance(0);
        bearing = getBearing(0);
    }

    public boolean isHealthy() {
        return aprilTagProcessor != null && visionPortal != null;
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData( getName() + " Distance", "%.0f Inches", distance);
        telemetry.addData(getName() + " Bearing", "%.0f Degrees", bearing);
        telemetry.addData(getName() + " Healthy", isHealthy());
    }

    /**
     * Get distance to tag with specified ID
     */
    public double getDistance(int ID) {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        // Step through the list of detections
        for (AprilTagDetection detection : currentDetections) {
            if (ID != 0) {
                if (detection.id == ID) { // If tag has the specified ID
                    return detection.ftcPose.range;
                }
            } else {
                if (
                        detection.id == Constants.Vision.BLUE_GOAL_ID
                        || detection.id == Constants.Vision.RED_GOAL_ID
                ) {
                    return detection.ftcPose.range;
                }
            }
        }   // end for() loop

        return -1.0; // Return tag data
    }

    /**
     * Get distance to tag with specified ID
     */
    public double getBearing(int ID) {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        // Step through the list of detections
        for (AprilTagDetection detection : currentDetections) {
            if (ID != 0) {
                if (detection.id == ID) { // If tag has the specified ID
                    return detection.ftcPose.bearing;
                }
            } else {
                if (
                        detection.id == Constants.Vision.BLUE_GOAL_ID
                        || detection.id == Constants.Vision.RED_GOAL_ID
                ) {
                    return detection.ftcPose.bearing;
                }
            }
        }   // end for() loop

        return -1.0;
    }

    /**
     * Set manual exposure and gain
     */
    public void setManualExposure() {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure((long) exposure, TimeUnit.MILLISECONDS);

            // Set Gain
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);

            // Get new exposure and gain values
            exposure = Constants.Vision.EXPOSURE;
            gain = Constants.Vision.GAIN;
        }
    }
}
