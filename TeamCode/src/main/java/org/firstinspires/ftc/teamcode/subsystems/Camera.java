package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.subsystems.settings.ConfigInfo;
import org.firstinspires.ftc.teamcode.subsystems.vision.Pipeline;
import org.firstinspires.ftc.teamcode.util.Mechanism;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Camera extends Mechanism {

    OpenCvCamera openCvCamera;
    Pipeline pipeline;

    private static VisionPortal visionPortal; // Used to manage the video source.
    private static AprilTagProcessor aprilTag; // Used for managing the AprilTag detection process
    private static AprilTagDetection detectedTag = null; // Used to hold the data for a detected AprilTag
    static final int exposureMS = 4; // ms exposure
    static final int gain = 250; // gain

    private static final int DESIRED_TAG_ID = -1; // The ID of the desired tag (-1 is all tags)
    private static TfodProcessor tfod; // Used for managing the TensorFlow Object detection process
    private static final String TFOD_MODEL_ASSET = "ssd_mobilenet_v2_320x320_coco17_tpu_8.tflite"; // Name of the model asset
    private static final String[] LABELS = { // Labels for the model
            "person",
            "bicycle",
            "car",
            "motorcycle",
            "airplane",
            "bus",
            "train",
            "truck",
            "boat",
            "traffic light",
            "fire hydrant",
            "???",
            "stop sign",
            "parking meter",
            "bench",
            "bird",
            "cat",
            "dog",
            "horse",
            "sheep",
            "cow",
            "elephant",
            "bear",
            "zebra",
            "giraffe",
            "???",
            "backpack",
            "umbrella",
            "???",
            "???",
            "handbag",
            "tie",
            "suitcase",
            "frisbee",
            "skis",
            "snowboard",
            "sports ball",
            "kite",
            "baseball bat",
            "baseball glove",
            "skateboard",
            "surfboard",
            "tennis racket",
            "bottle",
            "???",
            "wine glass",
            "cup",
            "fork",
            "knife",
            "spoon",
            "bowl",
            "banana",
            "apple",
            "sandwich",
            "orange",
            "broccoli",
            "carrot",
            "hot dog",
            "pizza",
            "donut",
            "cake",
            "chair",
            "couch",
            "potted plant",
            "bed",
            "???",
            "dining table",
            "???",
            "???",
            "toilet",
            "???",
            "tv",
            "laptop",
            "mouse",
            "remote",
            "keyboard",
            "cell phone",
            "microwave",
            "oven",
            "toaster",
            "sink",
            "refrigerator",
            "???",
            "book",
            "clock",
            "vase",
            "scissors",
            "teddy bear",
            "hair drier",
            "toothbrush"
    };

    private static final int LEFT_MAX_THRESHOLD = 213;
    private static final int CENTER_MAX_THRESHOLD = 427;

    private static final float MINIMUM_CONFIDENCE = 0.35f; // Minimum confidence for a detection to be considered valid

    private boolean isCustomModel; // Whether or not to use the custom model

    private WebcamName webcamName;

    /**
     * Constructor for Camera
     * @param isCustomModel whether or not to use the custom model
     */
    public Camera(boolean isCustomModel, boolean isRedAlliance) {
        this.isCustomModel = isCustomModel;
        pipeline = new Pipeline(isRedAlliance);
    }

    /**
     * Initializes the camera and vision processors.
     * @param hwMap references the robot's hardware map
     */
    @Override
    public void init(HardwareMap hwMap) {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcamName = hwMap.get(WebcamName.class, ConfigInfo.camera.getDeviceName());
        openCvCamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        openCam();
//        initAT();
//        initTFOD();
//        initVisionPortal(hwMap);
    }

    public void openCam() {
        openCvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                openCvCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                openCvCamera.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public void initAT() {
        aprilTag = new AprilTagProcessor.Builder()
                .build();
        aprilTag.setDecimation(3);
    }

    public void initTFOD() {
        if (isCustomModel) {
            tfod = new TfodProcessor.Builder()
                    .setModelAssetName(TFOD_MODEL_ASSET)
                    .setModelLabels(LABELS)
                    .build();
        } else {
            tfod = new TfodProcessor.Builder()
                    .build();
        }
        tfod.setMinResultConfidence(MINIMUM_CONFIDENCE);
    }

    public void initVisionPortal(HardwareMap hwMap) {
//        setManualExposure(exposureMS, gain);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, ConfigInfo.camera.getDeviceName()))
                .addProcessor(aprilTag)
                .addProcessor(tfod)
                .build();
    }

    /**
     * Updates the telemetry with the current camera data.
     * @param telemetry references local telemetry
     */
    @Override
    public void telemetry(Telemetry telemetry) {
//        telemetryTag(telemetry);
//        telemetryTfod(telemetry);
    }

    /**
     * Updates the camera exposure and gain settings
     * @param exposureMS the exposure in milliseconds
     * @param gain the gain
     */
    public void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) { // Check if the camera is streaming
            return;
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }

    public int whichRegion() {
        return pipeline.whichRegion();
    }

    public void stopStreaming() {
        openCvCamera.stopStreaming();
        openCvCamera.closeCameraDevice();
    }

    /**
     * Checks to see if the desired tag is detected and sets the targetFound variable and metadata accordingly
     */
    public void checkDetections() {
        List<AprilTagDetection> currentDetections = getDetections();
        boolean detectionFound = false; // Add this variable
        detectedTag = null;
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                // Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    detectedTag = detection;
                    detectionFound = true; // Set detectionFound to true
                    break;  // don't look any further.
                }
            }
        }
    }

    /**
     * Retruns the desired tag's pose data
     * @return pose data of the desired tag
     */
    public double[] getDesiredTagPoseData() {
        if (detectedTag != null) { // detectedTag is not null
            return new double[] {detectedTag.ftcPose.range, detectedTag.ftcPose.bearing, detectedTag.ftcPose.yaw};
        } else {
            return null;
        }
    }

    /**
     * Returns the camera state
     * @return the camera state
     */
    public VisionPortal.CameraState getCameraState() {
        return visionPortal.getCameraState();
    }

    /**
     * Returns the list of detections
     * @return the list of detections
     */
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    /**
     * Telemetry on desired tag pose data based on AprilTag detection
     * @param telemetry references local telemetry
     */
    private void telemetryTag(Telemetry telemetry) {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) { // Check if the camera is streaming
            if (detectedTag != null) {
                telemetry.addData("Tag ID ", detectedTag.id);
                telemetry.addData("Tag Name ", detectedTag.metadata.name);
                telemetry.addData("Tag X ", detectedTag.ftcPose.x);
                telemetry.addData("Tag Y ", detectedTag.ftcPose.y);
                telemetry.addData("Tag Z ", detectedTag.ftcPose.z);
                telemetry.addData("Tag Pitch ", detectedTag.ftcPose.pitch);
                telemetry.addData("Tag Roll ", detectedTag.ftcPose.roll);
                telemetry.addData("Tag Yaw ", detectedTag.ftcPose.yaw);
                telemetry.addData("Tag Range ", detectedTag.ftcPose.range);
                telemetry.addData("Tag Bearing ", detectedTag.ftcPose.bearing);
                telemetry.addData("Tag Elevation ", detectedTag.ftcPose.elevation);
            } else {
                telemetry.addLine("No TAG");
            }
        }
    }

    /**
     * Telemetry on TensorFlow Object Detection
     * @param telemetry references local telemetry
     */
    public void telemetryTfod(Telemetry telemetry) {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }

    /**
     * Returns the predicted position of the element based on TensorFlow Object Detection
     * @return randomization of the match
     */
    public int getTfodElementPos() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        for (Recognition recognition : currentRecognitions) {
            if (recognition.getLabel().equals(LABELS[0]) || recognition.getLabel().equals(LABELS[10]) || recognition.getLabel().equals(LABELS[85]) || recognition.getLabel().equals(LABELS[43])) { // TODO Update for detections
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;
                if (x < LEFT_MAX_THRESHOLD) {
                    return 1;
                } else if (x < CENTER_MAX_THRESHOLD) {
                    return 2;
                } else {
                    return 3;
                }
            }
        }
        return 2;
    }

    @Override
    public void systemsCheck(Gamepad gamepad, Telemetry telemetry) {
//        checkDetections();
//        telemetryTag(telemetry);
//        getTfodElementPos();
//        telemetryTfod(telemetry);
    }
}
