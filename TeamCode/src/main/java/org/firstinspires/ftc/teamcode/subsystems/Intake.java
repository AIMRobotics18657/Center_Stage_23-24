package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Mechanism;

/**
 * Intake class represents a specific mechanism on the robot.
 * It extends the Mechanism class and controls an intake motor (DcMotorEx).
 *
 * @author Nate Schmelkin
 */
public class Intake extends Mechanism {

    // DcMotorEx instance for the intake mechanism
    DcMotorEx intake;

    // Name identifier for the intake motor
    public String intakeName = "intake";

    /**
     * Initialize method for the Intake mechanism.
     * Gets the motor configuration from the hardware map.
     */
    @Override
    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotorEx.class, intakeName);
    }

    /**
     * Loop method to handle gamepad input for the Intake mechanism.
     * Currently, there is no specific action assigned in the loop method.
     *
     * @param gamepad references the gamepad for input
     */
    @Override
    public void loop(Gamepad gamepad) {
        // No specific action in the loop method for the Intake mechanism
    }

    /**
     * Turns on the intake at a specified speed.
     *
     * @param speed The speed at which the intake will operate. Always positive.
     */
    public void intake(double speed) {
        intake.setPower(Math.abs(speed));
    }

    /**
     * Turns on the outtake at a specified speed.
     *
     * @param speed The speed at which the outtake will operate. Always negative.
     */
    public void outtake(double speed) {
        intake.setPower(-Math.abs(speed));
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        intake.setPower(0);
    }
}
