package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.settings.GamepadSettings;
import org.firstinspires.ftc.teamcode.util.Mechanism;
import org.firstinspires.ftc.vision.VisionPortal;

public class Drivebase extends Mechanism {

    private static Pose2d STARTING_POS; // Starting position of the robot
    private final boolean resetIMU; // If the IMU should be reset
    private final boolean isRedAlliance; // If the robot is on the red alliance

    public Camera camera; // Camera object
    public MecanumDrive drive; // MecanumDrive object
    private IMU_Controller imu; // IMU object
    private double targetHeading; // Target heading for the robot

    private static final double AT_READ_MAX = 24.0; // Max distance to read AprilTags
    private boolean isSlowModeEnabled = false; // If the robot is in slow mode (Reduced strafe and forward speed. Slowed turn speed unless read by AT)

    /**
     * Constructor for Drivebase
     * @param startingPose Starting position of the robot
     */
    public Drivebase(boolean isRedAlliance, Pose2d startingPose, boolean resetIMU) {
        this.isRedAlliance = isRedAlliance;
        STARTING_POS = startingPose;
        this.resetIMU = resetIMU;
    }

    /**
     * Initializes the drivebase
     * @param hwMap references the robot's hardware map
     */
    @Override
    public void init(HardwareMap hwMap) {
        drive = new MecanumDrive(hwMap, STARTING_POS);
        camera = new Camera(true, isRedAlliance);
        imu = new IMU_Controller(hwMap, resetIMU);
        camera.init(hwMap);
        imu.init(hwMap);
        if (isRedAlliance) {
            targetHeading = -90;
        } else {
            targetHeading = 90;
        }
    }

    /**
     * Main loop of the drivebase
     * @param gamepad references gamepad in slot one
     */
    @Override
    public void loop(Gamepad gamepad) {
        drive(gamepad);
    }

    /**
     * Sends telemetry data to the driver station
     * @param telemetry references the driver station
     */
    @Override
    public void telemetry(Telemetry telemetry) {
        camera.telemetry(telemetry);
        telemetry.addData("DrivebaseSpeeding", isDriveSpeeding(new Gamepad()));
        telemetry.addData("Slow Mode Enabled", isSlowModeEnabled);
        telemetry.addData("Pose Data", camera.getDesiredTagPoseData());
        telemetry.addData("Current Detections", camera.getDetections());
    }

    /**
     * Drives the robot
     * @param gamepad references gamepad in slot one
     */
    public void drive(Gamepad gamepad) {
        isSlowModeEnabled = gamepad.right_trigger > GamepadSettings.GP1_TRIGGER_DEADZONE; // TODO: Change to a different button
        if (isSlowModeEnabled) {
            if (camera.getCameraState() == VisionPortal.CameraState.STREAMING) {
                camera.checkDetections();
            }
            drive.setDrivePowers(clampSpeedsToPoseVelocity(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x));
        } else {
            drive.setDrivePowers(inputToPoseVel(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x));
        }
    }

    /**
     * Converts input to a PoseVelocity2d object
     * @param y y-axis input
     * @param x x-axis input
     * @param rx rotation input
     * @return PoseVelocity2d pose velocity to be used by the drivebase
     */
    public PoseVelocity2d inputToPoseVel(double y, double x, double rx) {
        return new PoseVelocity2d(
                new Vector2d(
                        poweredInput(stickValWithDeadzone(y) * GamepadSettings.VY_WEIGHT),
                        poweredInput(stickValWithDeadzone(x) * GamepadSettings.VX_WEIGHT)
                ),
                poweredInput(stickValWithDeadzone(rx) * GamepadSettings.VRX_WEIGHT)
        );
    }

    /**
     * Clamps speeds to a PoseVelocity2d object based on the camera's pose data
     * @param y y-axis input
     * @param x x-axis input
     * @param rx rotation input
     * @return PoseVelocity2d pose velocity to be used by the drivebase
     */
    public PoseVelocity2d clampSpeedsToPoseVelocity(double y, double x, double rx) {
        final double YAW_KP = 0.016; // TODO: Tune this
        final double MAX_AUTO_TURN = 0.5; // TODO: Tune this
//        double turn = poweredInput(stickValWithDeadzone(rx) * GamepadSettings.VRX_WEIGHT_SLOW); // Set to slow mode turn speed
//        double[] poseData = camera.getDesiredTagPoseData();
//        if (poseData != null) { // If there is a tag detected
//            if (poseData[0] <= AT_READ_MAX) { // If the tag is close enough to read
//                double headingError = poseData[2]; // Get the heading error
//                turn = Range.clip(headingError * YAW_KP, -MAX_AUTO_TURN, MAX_AUTO_TURN); // Set the turn speed proportional to the error
//            }
//        }
        double headingError = imu.getHeadingError(targetHeading); // Get the heading error
        double turn = Range.clip(headingError * YAW_KP, -MAX_AUTO_TURN, MAX_AUTO_TURN) + poweredInput(stickValWithDeadzone(rx) * GamepadSettings.VRX_WEIGHT_SLOW); // Set the turn speed proportional to the error

        return new PoseVelocity2d( // Return the new pose velocity
                new Vector2d(
                        poweredInput(stickValWithDeadzone(y) * GamepadSettings.VY_WEIGHT_SLOW),
                        poweredInput(stickValWithDeadzone(x) * GamepadSettings.VX_WEIGHT_SLOW)
                ),
                turn
        );
    }

    /**
     * Returns the powered input
     * @param base base input
     * @return base to the EXPONENT_MODIFIER power
     */
    public double poweredInput(double base) {
        if (GamepadSettings.EXPONENT_MODIFIER % 2 == 0) {
            return Math.pow(base, GamepadSettings.EXPONENT_MODIFIER) * Math.signum(base);
        } else {
            return Math.pow(base, GamepadSettings.EXPONENT_MODIFIER);
        }
    }

    /**
     * Returns the stick value with a deadzone
     * @param stickVal stick value
     * @return stick value if it is greater than the deadzone, 0 otherwise
     */
    public double stickValWithDeadzone(double stickVal) {
        if (Math.abs(stickVal) > GamepadSettings.GP1_STICK_DEADZONE) {
            return stickVal;
        } else {
            return 0;
        }
    }

    /**
     * Returns the game randomization
     * @return camera's tfod element position
     */
    public int getGameRandomization() {
        return camera.getTfodElementPos();
    }

    /**
     * Returns if the drivebase is above the speeding constant
     * @return if the drivebase is speeding
     */
    public boolean isDriveSpeeding(Gamepad gamepad) {
        final double SPEEDING_CONST = 0.9;
        return Math.abs(gamepad.left_stick_y) > SPEEDING_CONST ||
                Math.abs(gamepad.left_stick_x) > SPEEDING_CONST ||
                Math.abs(gamepad.right_stick_x) > SPEEDING_CONST;
    }

    /**
     * Systems check for the drivebase
     */
    @Override
    public void systemsCheck(Gamepad gamepad, Telemetry telemetry) {
        drive(gamepad);
        telemetry(telemetry);
    }
}
