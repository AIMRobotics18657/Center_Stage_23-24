package org.firstinspires.ftc.teamcode.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardEx;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.LowPassEstimator;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Systems.BasicSystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import org.firstinspires.ftc.teamcode.util.Mechanism;
import org.firstinspires.ftc.teamcode.util.RunToPositionMotorUtil;

import java.util.function.DoubleSupplier;

/**
 * Slides class represents the subsystem controlling the linear slides on the robot.
 * It extends the Mechanism class and manages the movement of the slides using two DcMotorEx motors.
 *
 * @author Nate Schmelkin
 */

public class Slides extends Mechanism {

    // Util class for running motors to a specified position
    RunToPositionMotorUtil motorUtil = new RunToPositionMotorUtil();

    // DcMotorEx instances for left and right slides
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;

    // Variables to store the last positions of the left and right slides
    int lastLeftPos;
    int lastRightPos;

    // Speed limit for determining if the slides are moving too fast
    public double releaseSpeedLimit = 0.25;

    // Names for left and right slides in hardware configuration
    public String leftSlideName = "leftSlide";
    public String rightSlideName = "rightSlide";

    // Flags for tracking if the slides are moving too fast and if they are in the reset position
    public boolean isSpeeding = false;
    public boolean isReset = true;

    /**
     * Initializes the Slides subsystem by getting motor configurations from the hardware map.
     * Sets initial motor modes and behaviors.
     *
     * @param hwMap references the robot's hardware map
     */
    @Override
    public void init(HardwareMap hwMap) {
        leftSlide = hwMap.get(DcMotorEx.class, leftSlideName);
        rightSlide = hwMap.get(DcMotorEx.class, rightSlideName);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Main control loop for the Slides subsystem.
     * Checks if the slides are moving too fast and if they are in the reset position.
     *
     * @param gamepad references the gamepad for input
     */
    @Override
    public void loop(Gamepad gamepad) {
        isSpeeding = getAverageSlidesSpeed() > releaseSpeedLimit;
        isReset = getAverageSlidesPosition() == 0;
    }

    /**
     * Sets the power of both slides.
     *
     * @param power the power to set for both slides
     */
    public void setPower(double power) {
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        lastLeftPos = leftSlide.getCurrentPosition();
        lastRightPos = rightSlide.getCurrentPosition();
    }

    /**
     * Stops both slides by setting power to 0.
     */
    public void stop() {
        setPower(0);
    }

    /**
     * Sets the run mode for both slides.
     *
     * @param mode the run mode to set for both slides
     */
    public void setMode(DcMotor.RunMode mode) {
        leftSlide.setMode(mode);
        rightSlide.setMode(mode);
    }

    /**
     * Sets the zero power behavior for both slides.
     *
     * @param behavior the zero power behavior to set for both slides
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftSlide.setZeroPowerBehavior(behavior);
        rightSlide.setZeroPowerBehavior(behavior);
    }

    /**
     * Holds the position of both slides based on the last known positions.
     */
    public void holdPosition() {
        setSlidesPosition(lastLeftPos, lastRightPos, 1);
    }

    /**
     * Sets the target position and power for both slides.
     *
     * @param position the target position for both slides
     * @param speed    the power/speed for both slides
     */
    public void setSlidesPosition(int position, double speed) {
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorUtil.motorToPosition(leftSlide, 1, position);
        motorUtil.motorToPosition(rightSlide, 1, position);
    }

    /**
     * Sets the target positions and power for both slides independently.
     *
     * @param leftPosition  the target position for the left slide
     * @param rightPosition the target position for the right slide
     * @param speed         the power/speed for both slides
     */
    public void setSlidesPosition(int leftPosition, int rightPosition, double speed) {
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorUtil.motorToPosition(leftSlide, 1, leftPosition);
        motorUtil.motorToPosition(rightSlide, 1, rightPosition);
    }

    /**
     * Resets the slides to the zero position.
     *
     * @param speed the power/speed for both slides
     */
    public void resetSlidesPosition(double speed) {
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorUtil.motorToPosition(leftSlide, 1, 0);
        motorUtil.motorToPosition(rightSlide, 1, 0);
    }

    /**
     * Calculates and returns the average position of both slides.
     *
     * @return the average position of both slides
     */
    public int getAverageSlidesPosition() {
        return (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition()) / 2;
    }

    /**
     * Calculates and returns the average speed/power of both slides.
     *
     * @return the average speed/power of both slides
     */
    public double getAverageSlidesSpeed() {
        return (leftSlide.getPower() + rightSlide.getPower()) / 2;
    }
}
