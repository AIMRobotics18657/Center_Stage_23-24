package org.firstinspires.ftc.teamcode.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
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

/**
 * Drivebase class represents a specific mechanism on the robot.
 * It extends the Mechanism class and controls the MecanumDrive and Camera for movement and vision tasks.
 *
 * @author Nate Schmelkin
 */
public class Drivebase extends Mechanism {

    private static final double RANGE_KP = 0.02;
    private static final double YAW_KP = 0.03;

    private Camera camera;
    private MecanumDrive drive;

    private double speedClamp = 1;

    private static final double MAX_AUTO_SPEED = 0.65;
    private static final double MAX_AUTO_TURN = 0.5;

    private static final double SPEED_CLAMP_DIST = 8.0;
    private static final double DETECTION_ZONE = 24.0;

    private boolean isCameraControlling = false;

    /**
     * Initialize method for the Drivebase mechanism.
     * Initializes the MecanumDrive and Camera components.
     *
     * @param hwMap references the robot's hardware map
     */
    @Override
    public void init(HardwareMap hwMap) {
        drive = new MecanumDrive(hwMap, new Pose2d(0, 0, 0));
        camera = new Camera();
        camera.init(hwMap);
    }

    /**
     * Loop method to handle gamepad input and relate to corresponding hardware output.
     * Checks the camera state, updates the isCameraControlling flag, and sets drive powers.
     *
     * @param gamepad references the gamepad for input
     */
    @Override
    public void loop(Gamepad gamepad) {
        if (camera.getCameraState() == VisionPortal.CameraState.STREAMING) {
//            camera.checkAndSetDesiredTag(Camera.BLUE_CENTER_ID);
            camera.checkAndSetDesiredTag(-1);
        }
        updateIsCameraControlling(gamepad.a);
        drive.setDrivePowers(clampSpeeds(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x));
    }

    /**
     * Telemetry method to display relevant information.
     *
     * @param telemetry references local telemetry
     */
    @Override
    public void telemetry(Telemetry telemetry) {
        camera.telemetry(telemetry);
        telemetry.addData("speedClamp", speedClamp);
        telemetry.addData("Pose Data", camera.getDesiredTagPoseData());
        telemetry.addData("Target Found", camera.targetFound);
        telemetry.addData("Current Detections", camera.getDetections());
        telemetry.addData("isCameraControlling", isCameraControlling);
    }

    /**
     * Calculates powered input based on the gamepad stick value and exponent modifier.
     *
     * @param base the base value of the stick
     * @return the powered input
     */
    public double poweredInput(double base) {
        if (GamepadSettings.EXPONENT_MODIFIER % 2 == 0) {
            return Math.pow(base, GamepadSettings.EXPONENT_MODIFIER) * Math.signum(base);
        } else {
            return Math.pow(base, GamepadSettings.EXPONENT_MODIFIER);
        }
    }

    /**
     * Adjusts the stick value based on the deadzone.
     *
     * @param stickVal the stick value
     * @return the adjusted stick value
     */
    public double stickValWithDeadzone(double stickVal) {
        if (Math.abs(stickVal) > GamepadSettings.GP1_STICK_DEADZONE) {
            return stickVal;
        } else {
            return 0;
        }
    }

    /**
     * Calculates the distance to the speed clamp based on the given distance.
     *
     * @param distance the distance value
     * @return the calculated distance to the speed clamp
     */
    public double getDistanceToSpeedClamp(double distance) {
        return (distance - SPEED_CLAMP_DIST);
    }

    /**
     * Clamps the drive speeds based on camera information and gamepad input.
     *
     * @param y the y component of the drive velocity
     * @param x  the x component of the drive velocity
     * @param rx the rotation component of the drive velocity
     * @return the clamped PoseVelocity2d
     */
    public PoseVelocity2d clampSpeeds(double y, double x, double rx) {
        double straight;
        double turn;
        double[] poseData = camera.getDesiredTagPoseData();
        if (poseData != null) {
            if (getDistanceToSpeedClamp(poseData[0]) <= DETECTION_ZONE && y >= 0 && isCameraControlling) {
                double distanceError = getDistanceToSpeedClamp(poseData[0]);
                double headingError = poseData[2];
                straight = Range.clip(distanceError * RANGE_KP, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * YAW_KP, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                return new PoseVelocity2d(
                        new Vector2d(
                                straight,
                                poweredInput(stickValWithDeadzone(x) * GamepadSettings.VX_WEIGHT)
                        ),
                        turn
                );
            } else {
                isCameraControlling = false;
            }
        }
        return new PoseVelocity2d(
                new Vector2d(
                        poweredInput(stickValWithDeadzone(y) * GamepadSettings.VY_WEIGHT),
                        poweredInput(stickValWithDeadzone(x) * GamepadSettings.VX_WEIGHT)
                ),
                poweredInput(stickValWithDeadzone(rx) * GamepadSettings.VRX_WEIGHT)
        );
    }

    /**
     * Updates the isCameraControlling flag based on gamepad input and camera information.
     *
     * # @param isGamepadPressed true if the gamepad button is pressed, false otherwise
     */
    public void updateIsCameraControlling(boolean isGamepadPressed) {
        double[] poseData = camera.getDesiredTagPoseData();
        if (poseData != null) {
            if (getDistanceToSpeedClamp(poseData[0]) <= DETECTION_ZONE) {
                if (isGamepadPressed) {
                    isCameraControlling = true;
                }
            }
        }
    }
}