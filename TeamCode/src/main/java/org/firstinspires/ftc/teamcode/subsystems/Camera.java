package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.util.Mechanism;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

/**
 * Camera class represents a mechanism on the robot for vision processing using a webcam and AprilTags.
 * It extends the Mechanism class and provides methods to interact with the camera, process AprilTag detections, and set exposure settings.
 *
 * @author Nate Schmelkin
 */

public class Camera extends Mechanism {

    // VisionPortal instance to manage the video source
    private VisionPortal visionPortal;

    // AprilTagProcessor instance for managing the AprilTag detection process
    private AprilTagProcessor aprilTag;

    // AprilTagDetection instance to hold data for a detected AprilTag
    private AprilTagDetection desiredTag = null;

    // Flag to track whether a target has been found
    boolean targetFound = false;

    // Webcam name identifier
    private final String webcamName = "Ray";

    // Exposure settings for the camera
    private final int exposureMS = 4;
    private final int gain = 250;

    // Tag IDs for different positions
    public static final int BLUE_LEFT_ID = 1;
    public static final int BLUE_CENTER_ID = 2;
    public static final int BLUE_RIGHT_ID = 3;
    public static final int RED_LEFT_ID = 4;
    public static final int RED_CENTER_ID = 5;
    public static final int RED_RIGHT_ID = 6;

    /**
     * Initialize method for the Camera mechanism.
     * Builds AprilTagProcessor and VisionPortal instances, sets decimation, and configures exposure settings.
     *
     * @param hwMap references the robot's hardware map
     */
    @Override
    public void init(HardwareMap hwMap) {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, webcamName))
                .addProcessor(aprilTag)
                .build();
        aprilTag.setDecimation(3);
        setManualExposure(exposureMS, gain);
    }

    /**
     * Telemetry method to display information about the detected AprilTag.
     *
     * @param telemetry references local telemetry
     */
    @Override
    public void telemetry(Telemetry telemetry) {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            if (targetFound) {
                telemetry.addData("Tag ID ", desiredTag.id);
                telemetry.addData("Tag Name ", desiredTag.metadata.name);
                telemetry.addData("Tag X ", desiredTag.ftcPose.x);
                telemetry.addData("Tag Y ", desiredTag.ftcPose.y);
                telemetry.addData("Tag Z ", desiredTag.ftcPose.z);
                telemetry.addData("Tag Pitch ", desiredTag.ftcPose.pitch);
                telemetry.addData("Tag Roll ", desiredTag.ftcPose.roll);
                telemetry.addData("Tag Yaw ", desiredTag.ftcPose.yaw);
                telemetry.addData("Tag Range ", desiredTag.ftcPose.range);
                telemetry.addData("Tag Bearing ", desiredTag.ftcPose.bearing);
                telemetry.addData("Tag Elevation ", desiredTag.ftcPose.elevation);
            } else {
                telemetry.addLine("No TAG");
            }
        }
    }

    /**
     * Sets manual exposure settings for the camera.
     *
     * @param exposureMS desired exposure time in milliseconds
     * @param gain desired gain value
     */
    public void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return;
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }

    /**
     * Checks for and sets the desired AprilTag based on its ID.
     *
     * @param desiredTagID ID of the desired AprilTag
     */
    public void checkAndSetDesiredTag(int desiredTagID) {
        List<AprilTagDetection> currentDetections = getDetections();
        boolean detectionFound = false; // Add this variable

        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                // Check to see if we want to track towards this tag.
                if ((desiredTagID < 0) || (detection.id == desiredTagID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    detectionFound = true; // Set detectionFound to true
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    // You can remove this line, as it's not necessary.
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                // You can remove this line, as it's not necessary.
            }
        }

        // After the loop, set targetFound based on detectionFound
        if (!detectionFound) {
            targetFound = false;
        }
    }

    /**
     * Gets the pose data of the desired AprilTag.
     *
     * @return array containing range, bearing, and yaw if a target is found; null otherwise
     */
    public double[] getDesiredTagPoseData() {
        if (targetFound) {
            return new double[] {desiredTag.ftcPose.range, desiredTag.ftcPose.bearing, desiredTag.ftcPose.yaw};
        } else {
            return null;
        }
    }

    /**
     * Gets the camera state from the VisionPortal.
     *
     * @return camera state
     */
    public VisionPortal.CameraState getCameraState() {
        return visionPortal.getCameraState();
    }

    /**
     * Gets a list of AprilTag detections from the AprilTagProcessor.
     *
     * @return list of AprilTag detections
     */
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }
}
