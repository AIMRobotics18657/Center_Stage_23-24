package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.PIDSlides.FULL_EXTENSION_POS;
import static org.firstinspires.ftc.teamcode.subsystems.PIDSlides.HANGING_POS;
import static org.firstinspires.ftc.teamcode.subsystems.PIDSlides.RESET_POS;
import static org.firstinspires.ftc.teamcode.subsystems.PIDSlides.SAFE_EXTENSION_POS;
import static org.firstinspires.ftc.teamcode.subsystems.PIDSlides.SAFE_RETRACTION_POS;
import static org.firstinspires.ftc.teamcode.subsystems.PIDSlides.activeResetPos;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.settings.GamepadSettings;
import org.firstinspires.ftc.teamcode.util.Mechanism;
import org.opencv.core.Mat;

public class PixelManipulator extends Mechanism {

    public Arm arm;
    public Claw claw;
    public PIDSlides slides;

    double dropHeight = 0;


    enum ScoringState {
        PICKING_UP, PIXELSLOADED_ONE, PIXELSLOADED_TWO, POSITIONING, RELEASINGLEFT, RELEASINGRIGHT, RESETTING_STAGE_ONE, RESETTING_STAGE_TWO, FULL_MANUAL, HANGING
    }

    ScoringState activeScoringState = ScoringState.RESETTING_STAGE_TWO;
    boolean hangModeEnabled = false;

    @Override
    public void init(HardwareMap hwMap) {
        claw = new Claw();
        arm = new Arm();
        slides = new PIDSlides();

        claw.init(hwMap);
        arm.init(hwMap);
        slides.init(hwMap);
        arm.retract();
        setHangModeEnabled(false);
    }

    @Override
    public void loop(Gamepad gamepad, Gamepad gamepad2) {
        claw.loop(gamepad);
        arm.loop(gamepad);
        slides.loop(gamepad);

        boolean isReleasable = !slides.isSpeeding();

        boolean manualOverride = gamepad2.right_bumper && gamepad2.left_bumper && gamepad2.a; // TODO ADD IN BUTTON PRESSES
        boolean hangMode = gamepad2.right_bumper && gamepad2.left_bumper && gamepad2.dpad_up; // TODO ADD IN BUTTON PRESSES


        if (manualOverride) {
            setActiveScoringState(ScoringState.FULL_MANUAL);
        }
        if (hangMode) { // && hangModeEnabled
            setActiveScoringState(ScoringState.HANGING);
        }

        switch (activeScoringState) {
            case PICKING_UP:
                slides.update(slides.getActiveResetPos());
                if (gamepad2.y) {
                    claw.outtake();
                } else {
                    claw.intake();
                }
                if (gamepad2.a) {
                    setActiveScoringState(ScoringState.PIXELSLOADED_ONE);
                }
                break;

            case PIXELSLOADED_ONE:
                if (gamepad2.y) {
                    claw.outtake();
                }
                slides.update(SAFE_EXTENSION_POS);
                if (slides.isAtTargetPosition()) {
                    arm.extend();
                    claw.stopIntake();
                    setActiveScoringState(ScoringState.POSITIONING);
                }
                break;

//            case PIXELSLOADED_TWO:
//                arm.extend();
//                if (arm.isExtended && gamepad2.a) {
//                    claw.stopIntake();
//                    setActiveScoringState(ScoringState.POSITIONING);
//                }
//                break;


            case POSITIONING:
                if (Math.abs(gamepad2.left_stick_y) > GamepadSettings.GP2_STICK_DEADZONE) {
                    slides.setPower(gamepad2.left_stick_y);
                } else {
                    slides.holdPosition();
                }

                if (gamepad2.left_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE && isReleasable) {
                    setActiveScoringState(ScoringState.RELEASINGLEFT);
                } else if (gamepad2.right_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE && isReleasable) {
                    setActiveScoringState(ScoringState.RELEASINGRIGHT);
                }
                break;

            case RELEASINGLEFT:
                if (Math.abs(gamepad2.left_stick_y) > GamepadSettings.GP2_STICK_DEADZONE) {
                    slides.setPower(gamepad2.left_stick_y);
                } else {
                    slides.holdPosition();
                }

                claw.releaseServo(claw.leftClamp);
                if (claw.isLeftReleased && claw.isRightReleased) {
                    dropHeight = slides.getLastPosition();
                    activeScoringState = ScoringState.RESETTING_STAGE_ONE;
                }
                if (gamepad2.right_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE && isReleasable) {
                    setActiveScoringState(ScoringState.RELEASINGRIGHT);
                }
                break;

            case RELEASINGRIGHT:
                if (Math.abs(gamepad2.left_stick_y) > GamepadSettings.GP2_STICK_DEADZONE) {
                    slides.setPower(gamepad2.left_stick_y);
                } else {
                    slides.holdPosition();
                }

                claw.releaseServo(claw.rightClamp);
                if (claw.isLeftReleased && claw.isRightReleased) {
                    dropHeight = slides.getLastPosition();
                    activeScoringState = ScoringState.RESETTING_STAGE_ONE;
                }
                if (gamepad2.left_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE && isReleasable) {
                    setActiveScoringState(ScoringState.RELEASINGLEFT);
                }
                break;

            case RESETTING_STAGE_ONE:
                slides.update(Range.clip(dropHeight - 200, FULL_EXTENSION_POS, 0));
                if (slides.isAtTargetPosition()) {
                    arm.retract();
                    claw.clampServo(claw.leftClamp);
                    claw.clampServo(claw.rightClamp);
                    if (gamepad2.a) {
                        setActiveScoringState(ScoringState.RESETTING_STAGE_TWO);
                    }
                }
                break;

            case RESETTING_STAGE_TWO:
                slides.update(slides.getActiveResetPos());
                arm.retract();
                claw.clampServo(claw.leftClamp);
                claw.clampServo(claw.rightClamp);
                if (slides.isAtTargetPosition()) {
                    setActiveScoringState(ScoringState.PICKING_UP);
                }
                break;
            case FULL_MANUAL:
                if (Math.abs(gamepad2.left_stick_y) > GamepadSettings.GP2_STICK_DEADZONE) {
                    slides.setPower(gamepad2.left_stick_y);
                } else if (gamepad2.a) {
                    slides.update(SAFE_RETRACTION_POS);
                } else if (gamepad2.b) {
                    slides.update(SAFE_EXTENSION_POS);
                } else if (gamepad2.y) {
                    slides.update(slides.getActiveResetPos());
                } else {
                    slides.holdPosition();
                }

                if (gamepad2.right_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE) {
                    claw.intake();
                } else if (gamepad2.left_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE) {
                    claw.outtake();
                } else {
                    claw.stopIntake();
                }

                if (gamepad2.dpad_up) {
                    arm.extend();
                } else if (gamepad2.dpad_down) {
                    arm.retract();
                }
            case HANGING:
                arm.retract();
                claw.stopIntake();
                if (gamepad2.dpad_up) {
                    slides.update(FULL_EXTENSION_POS);
                } else if (gamepad2.dpad_down) {
                    slides.update(slides.getActiveResetPos());
                } else if (gamepad2.dpad_left) {
                    slides.update(HANGING_POS);
                }

        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        claw.telemetry(telemetry);
        arm.telemetry(telemetry);
        slides.telemetry(telemetry);
        telemetry.addData("Active Scoring State", activeScoringState);
    }

    public void setActiveScoringState(ScoringState state) {
        activeScoringState = state;
    }

    public void setHangModeEnabled(boolean hangModeEnabled) {
        this.hangModeEnabled = hangModeEnabled;
    }
}