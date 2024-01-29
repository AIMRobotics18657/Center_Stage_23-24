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

public class Drivebase extends Mechanism {

    private static Pose2d STARTING_POS;
    private static Position startingPosition;

    public Camera camera;
    public MecanumDrive drive;
//    public AutoLocalizer localizer;

    private static final double DETECTION_ZONE = 24.0;
    private boolean isCameraControlling = false;

    //
    // AUTO VARS
    //

    private final double axialKp = 0.05; // TODO: Tune
    private final double axialKi = 0.0;
    private final double axialKd = 0.0;
    private final double axialIntegralSumMax = 0.0;
    private final double axialStabilityThreshold = 0.0;
    private final double axialLowPassGain = 0.0;


    private final double lateralKp = 0.05;
    private final double lateralKi = 0.0;
    private final double lateralKd = 0.0;
    private final double lateralIntegralSumMax = 0.0;
    private final double lateralStabilityThreshold = 0.0;
    private final double lateralLowPassGain = 0.0;


    private double headingKp = 0.05;
    private double headingKi = 0.0;
    private final double headingKd = 0.0;
    private final double headingIntegralSumMax = 0.0;
    private final double headingStabilityThreshold = 0.0;
    private final double headingLowPassGain = 0.0;

    private PIDEx axialPID = new PIDEx(new PIDCoefficientsEx(axialKp, axialKi, axialKd, axialIntegralSumMax, axialStabilityThreshold, axialLowPassGain));
    private PIDEx lateralPID = new PIDEx(new PIDCoefficientsEx(lateralKp, lateralKi, lateralKd, lateralIntegralSumMax, lateralStabilityThreshold, lateralLowPassGain));
    private PIDEx headingPID = new PIDEx(new PIDCoefficientsEx(headingKp, headingKi, headingKd, headingIntegralSumMax, headingStabilityThreshold, headingLowPassGain));

    private static final double XY_CLOSE_THRESHOLD = 0.1;
    private static final double HEADING_CLOSE_THRESHOLD = 0.5;

    public Drivebase(Pose2d startingPose, Position startingPosition) {
        STARTING_POS = startingPose;
        this.startingPosition = startingPosition;
    }

    @Override
    public void init(HardwareMap hwMap) {
//        localizer = new AutoLocalizer(1000, 1000, startingPosition, hwMap);
        drive = new MecanumDrive(hwMap, STARTING_POS);
        camera = new Camera(true);
        camera.init(hwMap);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (camera.getCameraState() == VisionPortal.CameraState.STREAMING) {
//            camera.checkAndSetDesiredTag(Camera.BLUE_CENTER_ID);
            camera.checkAndSetDesiredTag(-1);
        }
        updateIsCameraControlling(gamepad.a);
//        drive.setDrivePowers(clampSpeeds(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x));
        drive.setDrivePowers(inputToPoseVel(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x));
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        camera.telemetry(telemetry);
        telemetry.addData("DrivebaseSpeeding", isDriveSpeeding());
        telemetry.addData("Pose Data", camera.getDesiredTagPoseData());
        telemetry.addData("Target Found", camera.targetFound);
        telemetry.addData("Current Detections", camera.getDetections());
        telemetry.addData("isCameraControlling", isCameraControlling);
    }

    public boolean isDriveSpeeding() {
        final double SPEEDING_CONST = 0.1;
        return drive.leftBack.getPower() > SPEEDING_CONST ||
                drive.leftFront.getPower() > SPEEDING_CONST ||
                drive.rightBack.getPower() > SPEEDING_CONST ||
                drive.rightFront.getPower() > SPEEDING_CONST;
    }

    public double poweredInput(double base) {
        if (GamepadSettings.EXPONENT_MODIFIER % 2 == 0) {
            return Math.pow(base, GamepadSettings.EXPONENT_MODIFIER) * Math.signum(base);
        } else {
            return Math.pow(base, GamepadSettings.EXPONENT_MODIFIER);
        }
    }

    public double stickValWithDeadzone(double stickVal) {
        if (Math.abs(stickVal) > GamepadSettings.GP1_STICK_DEADZONE) {
            return stickVal;
        } else {
            return 0;
        }
    }

    public double getDistanceToAprilControlZone(double distance) {
        final double APRIL_CONTROL_ZONE_DIST = 4.0;
        return (distance - APRIL_CONTROL_ZONE_DIST);
    }

    public PoseVelocity2d clampSpeeds(double y, double x, double rx) {
        final double RANGE_KP = 0.02;
        final double YAW_KP = 0.03;
        final double MAX_AUTO_SPEED = 0.65;
        final double MAX_AUTO_TURN = 0.5;
        double straight;
        double turn;
        double[] poseData = camera.getDesiredTagPoseData();
        if (poseData != null) {
            if (getDistanceToAprilControlZone(poseData[0]) <= DETECTION_ZONE && y >= 0 && isCameraControlling) {
                double distanceError = getDistanceToAprilControlZone(poseData[0]);
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
        return inputToPoseVel(y, x, rx);
    }

    public PoseVelocity2d inputToPoseVel(double y, double x, double rx) {
        return new PoseVelocity2d(
                new Vector2d(
                        poweredInput(stickValWithDeadzone(y) * GamepadSettings.VY_WEIGHT),
                        poweredInput(stickValWithDeadzone(x) * GamepadSettings.VX_WEIGHT)
                ),
                poweredInput(stickValWithDeadzone(rx) * GamepadSettings.VRX_WEIGHT)
        );
    }

    public void updateIsCameraControlling(boolean isGamepadPressed) {
        double[] poseData = camera.getDesiredTagPoseData();
        if (poseData != null) {
            if (getDistanceToAprilControlZone(poseData[0]) <= DETECTION_ZONE) {
                if (isGamepadPressed) {
                    isCameraControlling = true;
                }
            }
        }
    }

    public int getGameRandomization() {
        return camera.getTfodElementPos();
    }

    @Override
    public void systemsCheck(Gamepad gamepad, Telemetry telemetry) {
        drive.setDrivePowers(inputToPoseVel(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x));
        telemetry.addData("Pose Data", camera.getDesiredTagPoseData());
        telemetry.addData("Target Found", camera.targetFound);
        telemetry.addData("Current Detections", camera.getDetections());
        telemetry.addData("isCameraControlling", isCameraControlling);
    }

    //
    // AUTO METHODS
    //

//    public void driveTo(double targetX, double targetY, double targetHeading) {
//        while (!isAtPosition(targetX, targetY, targetHeading)) { // TODO: Distance greater than certain amount or custom timeout specified
//            localizer.calculateCurrentPose();
//            drive.setDrivePowers(inputToPoseVel(
//                    axialPID.calculate(targetX, localizer.getCurrentPosition().getX()),
//                    lateralPID.calculate(targetY, localizer.getCurrentPosition().getY()),
//                    headingPID.calculate(targetHeading, localizer.getCurrentPosition().getHeading())));
//        } // Target, current
//    }
//
//    public boolean isAtPosition(double targetX, double targetY, double targetHeading) {
//        return (localizer.getCurrentPosition().getX() - targetX) < XY_CLOSE_THRESHOLD && (localizer.getCurrentPosition().getY() - targetY) < XY_CLOSE_THRESHOLD && (localizer.getCurrentPosition().getHeading() - targetHeading) < HEADING_CLOSE_THRESHOLD;
//    }
}
