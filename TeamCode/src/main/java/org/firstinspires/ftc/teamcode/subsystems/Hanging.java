package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Mechanism;

/**
 * Hanging class represents a specific mechanism on the robot.
 * It extends the Mechanism class and controls a hanging mechanism using a DcMotor.
 *
 * @author Dahlia Zemmel
 */
public class Hanging extends Mechanism {

    // DcMotor instance for the hanging mechanism
    DcMotor Hanger;

    // Name identifier for the hanging motor
    public String HangerName = "Hanger";

    /**
     * Initialize method for the Hanging mechanism.
     * Gets the motor configuration from the hardware map.
     *
     * @param hwMap references the robot's hardware map
     */
    @Override
    public void init(HardwareMap hwMap) {
        Hanger = hwMap.get(DcMotor.class, HangerName);
    }

    /**
     * Loop method to handle gamepad input and relate to corresponding motor output.
     * Sets the power of the hanging motor based on gamepad input.
     *
     * @param gamepad references the gamepad for input
     */
    @Override
    public void loop(Gamepad gamepad) {
        // Check if the right bumper is pressed, set motor power to 1
        if (gamepad.right_bumper) {
            Hanger.setPower(1);
        }
        // Check if the left bumper is pressed, set motor power to -1
        else if (gamepad.left_bumper) {
            Hanger.setPower(-1);
        }
        // If neither bumper is pressed, stop the motor
        else {
            Hanger.setPower(0);
        }
    }
}
