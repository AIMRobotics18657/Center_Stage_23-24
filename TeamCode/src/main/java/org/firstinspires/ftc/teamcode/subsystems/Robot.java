// Robot class representing the overall robot control

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Mechanism;

/**
 * Robot class manages the main robot control by incorporating PixelManipulator and Drivebase subsystems.
 * Extends the Mechanism class.
 *
 * @author Nate Schmelkin
 */
public class Robot extends Mechanism {

    // Subsystem instances
    PixelManipulator pixelManipulator;
    Drivebase drivebase;

    // Initialize subsystems during robot initialization
    @Override
    public void init(HardwareMap hwMap) {
        pixelManipulator = new PixelManipulator();
        drivebase = new Drivebase();
    }

    // Delegate controller input to PixelManipulator and Drivebase during the loop
    @Override
    public void loop(Gamepad gamepad, Gamepad gamepad2) {
        pixelManipulator.loop(gamepad, gamepad2);
        drivebase.loop(gamepad);
    }

    // Display telemetry data from PixelManipulator and Drivebase
    @Override
    public void telemetry(Telemetry telemetry) {
        pixelManipulator.telemetry(telemetry);
        drivebase.telemetry(telemetry);
    }
}
