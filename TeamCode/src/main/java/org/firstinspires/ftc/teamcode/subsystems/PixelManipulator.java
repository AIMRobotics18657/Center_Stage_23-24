package org.firstinspires.ftc.teamcode.subsystems;

import com.ThermalEquilibrium.homeostasis.Utils.WPILibMotionProfile;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.developmental.PIDSlides;
import org.firstinspires.ftc.teamcode.subsystems.settings.GamepadSettings;
import org.firstinspires.ftc.teamcode.util.Mechanism;

/**
 * PixelManipulator class represents a complex mechanism on the robot.
 * It extends the Mechanism class and controls multiple subsystems, including Arm, Claw, Intake, and PIDSlides.
 * The class manages the state machine for different stages of pixel manipulation.
 *
 * @author Nate Schmelkin
 */
public class PixelManipulator extends Mechanism {

    // Instances of the subsystems
    Arm arm;
    Claw claw;
    Intake intake;
    PIDSlides slides;

    // Enumeration to represent different states during pixel manipulation
    enum ScoringState {
        PICKINGUP, PIXELSLOADED, POSITIONING, RELEASINGLEFT, RELEASINGRIGHT, RESETTING_STAGE_ONE, RESETTING_STAGE_TWO
    }

    // Current active scoring state
    ScoringState activeScoringState = ScoringState.RESETTING_STAGE_TWO;

    /**
     * Initialize method for the PixelManipulator mechanism.
     * Initializes instances of the subsystems (Arm, Claw, Intake, PIDSlides).
     *
     * @param hwMap references the robot's hardware map
     */
    @Override
    public void init(HardwareMap hwMap) {
        // Initialize subsystem instances
        intake = new Intake();
        claw = new Claw();
        arm = new Arm();
        slides = new PIDSlides();

        // Initialize subsystems with the hardware map
        intake.init(hwMap);
        claw.init(hwMap);
        arm.init(hwMap);
        slides.init(hwMap);
    }


    /**
     * Implements the control logic for the PixelManipulator mechanism.
     *
     * @param gamepad  The primary gamepad for input.
     * @param gamepad2 The secondary gamepad for additional input.
     */
    @Override
    public void loop(Gamepad gamepad, Gamepad gamepad2) {
        // Update each subsystem based on input
        intake.loop(gamepad);
        claw.loop(gamepad);
        arm.loop(gamepad);
        slides.loop(gamepad);

        // Check conditions for various states
        boolean isPixelsLoaded = claw.isLeftClamped && claw.isRightClamped;
        boolean isReleasable = claw.isRotatorInPosition && !slides.isSpeeding();

        // State machine for scoring actions
        switch (activeScoringState) {
            case PICKINGUP:
                intake.intake(1);
                if (gamepad2.left_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE) {
                    claw.clampServo(claw.leftProng);
                } else if (gamepad2.right_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE) {
                    claw.clampServo(claw.leftProng);
                }
                if (isPixelsLoaded && gamepad2.a) {
                    setActiveScoringState(ScoringState.PIXELSLOADED);
                }
                break;

            case PIXELSLOADED:
                intake.stop();
                slides.update(PIDSlides.SAFE_EXTENSION_POS);
                if (slides.isAtTargetPosition()) {
                    arm.extend();
                    if (arm.isExtended && gamepad2.a) {
                        setActiveScoringState(ScoringState.POSITIONING);
                    }
                }
                break;

            case POSITIONING:
                if (Math.abs(gamepad2.left_stick_y) > GamepadSettings.GP2_STICK_DEADZONE) {
                    slides.setPower(gamepad2.left_stick_y);
                } else {
                    slides.holdPosition();
                }

                if (gamepad2.x) {
                    claw.setActiveTiltState(Claw.TiltState.LEFT);
                } else if (gamepad2.y) {
                    claw.setActiveTiltState(Claw.TiltState.CENTER);
                } else if (gamepad2.b) {
                    claw.setActiveTiltState(Claw.TiltState.RIGHT);
                }
                if (gamepad2.left_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE && isReleasable) {
                    setActiveScoringState(ScoringState.RELEASINGLEFT);
                } else if (gamepad2.right_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE && isReleasable) {
                    setActiveScoringState(ScoringState.RELEASINGRIGHT);
                }
                break;

            case RELEASINGLEFT:
                claw.releaseServo(claw.leftProng);
                if (claw.isLeftReleased && claw.isRightReleased) {
                    activeScoringState = ScoringState.RESETTING_STAGE_ONE;
                }
                if (gamepad2.right_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE && isReleasable) {
                    setActiveScoringState(ScoringState.RELEASINGRIGHT);
                }
                break;

            case RELEASINGRIGHT:
                claw.releaseServo(claw.rightProng);
                if (claw.isLeftReleased && claw.isRightReleased) {
                    activeScoringState = ScoringState.RESETTING_STAGE_ONE;
                }
                if (gamepad2.left_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE && isReleasable) {
                    setActiveScoringState(ScoringState.RELEASINGLEFT);
                }
                break;

            case RESETTING_STAGE_ONE:
                claw.setActiveTiltState(Claw.TiltState.CENTER);
                slides.update(PIDSlides.SAFE_RETRACTION_POS);
                if (slides.isAtTargetPosition() && claw.isRotatorInPosition && gamepad.a) {
                    setActiveScoringState(ScoringState.RESETTING_STAGE_TWO);
                }
                break;

            case RESETTING_STAGE_TWO:
                arm.retract();
                if (arm.isRetracted && gamepad2.a) {
                    slides.resetSlidesPosition();
                    if (slides.isAtTargetPosition()) {
                        setActiveScoringState(ScoringState.PICKINGUP);
                    }
                }
        }
    }

    /**
     * Sets the active scoring state for the PixelManipulator.
     *
     * @param state The desired scoring state.
     */
    public void setActiveScoringState(ScoringState state) {
        activeScoringState = state;
    }
}