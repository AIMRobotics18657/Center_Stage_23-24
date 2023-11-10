package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Mechanism;

/**
 * The clawDustpan class represents a mechanism on the robot for controlling a servo-based claw or dustpan.
 * It extends the Mechanism class and provides methods for initialization and handling input to control the servo.
 *
 * @author Dahlia Zemmel
 */

public class clawDustpan extends Mechanism {

    // Servo instance for the claw or dustpan
    Servo clawDustpan;

    // Name identifier for the servo
    public String intakeName = "clawDustpan";

    /**
     * Initialize method for the ClawDustpan mechanism.
     * Gets the servo configuration from the hardware map.
     *
     * @param hwMap references the robot's hardware map
     */
    @Override
    public void init(HardwareMap hwMap) {
        clawDustpan = hwMap.get(Servo.class, intakeName);
    }

    /**
     * Loop method to handle gamepad input and relate to corresponding servo positions.
     * Uses the 'A' button to set the servo position to 1, the 'B' button to set it to -1, and sets it to 0 otherwise.
     *
     * @param gamepad references the gamepad for input
     */
    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.a) {
            // If the 'A' button is pressed, set the servo position to 1
            clawDustpan.setPosition(1);
        } else if (gamepad.b) {
            // If the 'B' button is pressed, set the servo position to -1
            clawDustpan.setPosition(-1);
        } else {
            // If neither 'A' nor 'B' is pressed, set the servo position to 0
            clawDustpan.setPosition(0);
        }
    }
}
