package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Mechanism;

/**
 * PAL class represents a specific mechanism on the robot.
 * It extends the Mechanism class and is designed to control a servo named "pal."
 *
 * @author Julian Ghiazza
 */

public class PAL extends Mechanism {

    // Servo instance for the PAL mechanism
    Servo pal;

    // Name identifier for the PAL servo
    public String palname = "pal";

    // Flag to track whether a time duration has elapsed
    boolean timeDuration = false;

    // ElapsedTime object to measure time duration
    public ElapsedTime runtime = new ElapsedTime();

    /**
     * Initialize method for the PAL mechanism.
     * Gets the servo configuration from the hardware map.
     *
     * @param hwMap references the robot's hardware map
     */
    @Override
    public void init(HardwareMap hwMap) {
        pal = hwMap.get(Servo.class, palname);
    }

    /**
     * Begins the timer by resetting the ElapsedTime object.
     */
    public void beginTimer() {
        runtime.reset();
    }

    /**
     * Gets the elapsed time since the timer started.
     * If the elapsed time exceeds 30 seconds, sets the timeDuration flag to true.
     *
     * @return elapsed time in seconds
     */
    public double getElapsedTime() {
        double elapsedTime = runtime.seconds();
        if (elapsedTime >= 30) {
            timeDuration = true;
        }
        return elapsedTime;
    }
}