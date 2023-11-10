package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Mechanism;

/**
 * Arm class represents a specific mechanism on the robot.
 * It extends the Mechanism class and is designed to control two servos, leftArm and rightArm, for an arm-like mechanism.
 *
 * @author Nate Schmelkin
 */

public class Arm extends Mechanism {

    // Servo instances for the left and right arms
    public Servo leftArm;
    public Servo rightArm;

    // Name identifiers for the left and right arm servos
    public String leftName = "leftArm";
    public String rightName = "rightArm";

    // Default positions for the extended and retracted states of the arm
    double extendedPosition = 0.3;
    double retractedPosition = 0.09;

    // Flags to track whether the arm is in an extended or retracted state
    public boolean isExtended = false;
    public boolean isRetracted = false;

    /**
     * Initialize method for the Arm mechanism.
     * Gets the servo configurations from the hardware map and sets the direction of the left arm servo to REVERSE.
     *
     * @param hwMap references the robot's hardware map
     */
    @Override
    public void init(HardwareMap hwMap) {
        leftArm = hwMap.get(Servo.class, leftName);
        rightArm = hwMap.get(Servo.class, rightName);
        leftArm.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * Loop method to handle gamepad input and relate to corresponding hardware output.
     * Checks if the arm is in an extended or retracted state based on servo positions.
     *
     * @param gamepad references the gamepad for input
     */
    @Override
    public void loop(Gamepad gamepad) {
        isExtended = leftArm.getPosition() == extendedPosition && rightArm.getPosition() == extendedPosition;
        isRetracted = leftArm.getPosition() == retractedPosition && rightArm.getPosition() == retractedPosition;
    }

    /**
     * Telemetry method to display arm servo positions.
     *
     * @param telemetry references local telemetry
     */
    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Left Arm Position:", leftArm.getPosition());
        telemetry.addData("Right Arm Position:", rightArm.getPosition());
    }

    /**
     * Method to extend the arm by setting both servos to the extended position.
     */
    public void extend() {
        leftArm.setPosition(extendedPosition);
        rightArm.setPosition(extendedPosition);
    }

    /**
     * Method to retract the arm by setting both servos to the retracted position.
     */
    public void retract() {
        leftArm.setPosition(retractedPosition);
        rightArm.setPosition(retractedPosition);
    }
}




