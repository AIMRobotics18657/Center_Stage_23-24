package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Mechanism;

/**
 * Claw class represents a mechanism on the robot for controlling a claw with servos.
 * It extends the Mechanism class and provides methods to tilt, clamp, and release the claw, as well as set the active tilt state.
 *
 * @author Nate Schmelkin
 */

public class Claw extends Mechanism {

    // Servo instances for the rotator, left prong, and right prong of the claw
    public Servo rotator;
    public Servo leftProng;
    public Servo rightProng;

    // Name identifiers for the servos
    public String rotatorName = "rotator";
    public String leftProngName = "leftProng";
    public String rightProngName = "rightProng";

    // Servo positions for different tilt states
    public double tiltedRightPos = 0.77;
    public double tiltedLeftPos = 0.27;
    public double straight = 0.52;
    public double releasePosition = 0.1;
    public double clampPosition = 0;

    // Flags to track the state of the left and right prongs
    public boolean isLeftClamped = false;
    public boolean isLeftReleased = true;
    public boolean isRightClamped = false;
    public boolean isRightReleased = true;

    // Flag to track whether the rotator is in the target position
    public boolean isRotatorInPosition = false;

    // Enum to represent different tilt states
    public enum TiltState {
        LEFT, RIGHT, CENTER
    }

    // Active tilt state
    TiltState activeTiltState = TiltState.CENTER;

    // Target tilt position
    public double targetTilt = 0;

    /**
     * Initialize method for the Claw mechanism.
     * Gets the servo configurations from the hardware map and sets the direction of the left prong servo to REVERSE.
     *
     * @param hwMap references the robot's hardware map
     */
    @Override
    public void init(HardwareMap hwMap) {
        rotator = hwMap.get(Servo.class, rotatorName);
        leftProng = hwMap.get(Servo.class, leftProngName);
        rightProng = hwMap.get(Servo.class, rightProngName);
        leftProng.setDirection(Servo.Direction.REVERSE);
        targetTilt = straight;
    }

    /**
     * Loop method to handle gamepad input and relate to corresponding hardware output.
     * Sets the target tilt based on the active tilt state and checks the positions of servos.
     *
     * @param gamepad references the gamepad for input
     */
    @Override
    public void loop(Gamepad gamepad) {
        switch (activeTiltState) {
            case LEFT:
                targetTilt = tiltedLeftPos;
                tilt(tiltedLeftPos);
                break;
            case RIGHT:
                targetTilt = tiltedRightPos;
                tilt(tiltedRightPos);
                break;
            case CENTER:
                targetTilt = straight;
                tilt(straight);
                break;
        }

        isRotatorInPosition = rotator.getPosition() == targetTilt;

        isLeftClamped = leftProng.getPosition() == clampPosition;
        isRightClamped = rightProng.getPosition() == clampPosition;
        isLeftReleased = leftProng.getPosition() == releasePosition;
        isRightReleased = rightProng.getPosition() == releasePosition;
    }

    /**
     * Telemetry method to display information about the Claw mechanism.
     *
     * @param telemetry references local telemetry
     */
    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Rotator Position", rotator.getPosition());
        telemetry.addData("Left Prong Position", leftProng.getPosition());
        telemetry.addData("Right Prong Position", rightProng.getPosition());
        telemetry.addData("Target Tilt", targetTilt);
        telemetry.addData("Active Tilt State", activeTiltState);
        telemetry.addData("isRotatorInPosition", isRotatorInPosition);
        telemetry.addData("isLeftClamped", isLeftClamped);
        telemetry.addData("isRightClamped", isRightClamped);
        telemetry.addData("isLeftReleased", isLeftReleased);
        telemetry.addData("isRightReleased", isRightReleased);
    }

    /**
     * Method to set the tilt position of the rotator servo.
     *
     * @param position desired tilt position
     */
    public void tilt(double position) {
        rotator.setPosition(position);
    }

    /**
     * Method to set the specified servo to the clamp position.
     *
     * @param servo servo to be clamped
     */
    public void clampServo(Servo servo) {
        servo.setPosition(clampPosition);
    }

    /**
     * Method to set the specified servo to the release position.
     *
     * @param servo servo to be released
     */
    public void releaseServo(Servo servo) {
        servo.setPosition(releasePosition);
    }

    /**
     * Method to set the active tilt state of the Claw mechanism.
     *
     * @param tiltState desired tilt state (LEFT, RIGHT, CENTER)
     */
    public void setActiveTiltState(TiltState tiltState) {
        activeTiltState = tiltState;
    }
}