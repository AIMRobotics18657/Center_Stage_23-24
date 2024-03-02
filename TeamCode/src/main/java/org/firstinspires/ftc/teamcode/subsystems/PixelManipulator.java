package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.PIDSlides.FULL_EXTENSION_POS;
import static org.firstinspires.ftc.teamcode.subsystems.PIDSlides.HANGING_POS;
import static org.firstinspires.ftc.teamcode.subsystems.PIDSlides.MIN_EXTENSION_POS;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.settings.GamepadSettings;
import org.firstinspires.ftc.teamcode.util.Mechanism;

public class PixelManipulator extends Mechanism {

    public Arm arm;
    public Claw claw;
    public PIDSlides slides;

    double lastDropHeight = 0;
    double reZeroPos = 0;


    enum ScoringState {
        PICKING_UP, PIXELSLOADED_ONE, POSITIONING, RELEASINGLEFT, RELEASINGRIGHT, RESETTING_STAGE_ONE, RESETTING_STAGE_TWO, FULL_MANUAL, HANGING, REZEROING_1, REZEROING_2
    }

    enum HangingState {
        RESET, HANG, MAX
    }

    ScoringState activeScoringState = ScoringState.RESETTING_STAGE_TWO;
    HangingState activeHangingState = HangingState.HANG;
    boolean hangModeEnabled = false;

    @Override
    public void init(HardwareMap hwMap) {
        claw = new Claw();
        arm = new Arm();
        slides = new PIDSlides();

        claw.init(hwMap);
        arm.init(hwMap);
        slides.init(hwMap);
        arm.setRetractPos();
        arm.retract();
        setHangModeEnabled(false);
    }

    @Override
    public void loop(Gamepad gamepad, Gamepad gamepad2) {
        claw.loop(gamepad);
        arm.loop(gamepad);
        slides.loop(gamepad);

        boolean isReleasable = !slides.isSpeeding();

        boolean manualOverride = gamepad2.right_bumper && gamepad2.left_bumper && gamepad2.a;
        boolean hangMode = (gamepad2.right_bumper && gamepad2.left_bumper && gamepad2.dpad_up && hangModeEnabled) || (gamepad2.right_bumper && gamepad2.left_bumper && gamepad2.dpad_up && gamepad2.y);


        if (manualOverride) {
            setActiveScoringState(ScoringState.FULL_MANUAL);
        } else if (hangMode) {
            setActiveScoringState(ScoringState.HANGING);
        }

        switch (activeScoringState) {
            case PICKING_UP:
                slides.update(slides.getActiveResetPos());
                if (gamepad2.y || gamepad.a) {
                    claw.outtake();
                } else {
                    claw.intake();
                }
                if (gamepad2.a) {
                    setActiveScoringState(ScoringState.PIXELSLOADED_ONE);
                }

                if (gamepad.start && gamepad.options) {
                    activeScoringState = ScoringState.REZEROING_1;
                }
                break;

            case PIXELSLOADED_ONE:
                if (gamepad.a || gamepad2.y) {
                    claw.outtake();
                }
                slides.update(Range.clip(lastDropHeight, FULL_EXTENSION_POS, MIN_EXTENSION_POS));
                if (slides.isAtTargetPosition()) {
                    arm.extend();
                    setActiveScoringState(ScoringState.POSITIONING);
                }
                break;

            case POSITIONING:
                if (Math.abs(gamepad2.left_stick_y) > GamepadSettings.GP2_STICK_DEADZONE) {
                    slides.setPower(gamepad2.left_stick_y);
                } else if (gamepad2.x) {
                    slides.update(MIN_EXTENSION_POS);
                }
                else {
                    slides.holdPosition();
                }

                if (gamepad2.left_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE && isReleasable) {
                    claw.stopIntake();
                    setActiveScoringState(ScoringState.RELEASINGLEFT);
                } else if (gamepad2.right_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE && isReleasable) {
                    claw.stopIntake();
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
                    lastDropHeight = slides.getLastPosition();
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
                    lastDropHeight = slides.getLastPosition();
                    activeScoringState = ScoringState.RESETTING_STAGE_ONE;
                }
                if (gamepad2.left_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE && isReleasable) {
                    setActiveScoringState(ScoringState.RELEASINGLEFT);
                }
                break;

            case RESETTING_STAGE_ONE:
                slides.update(Range.clip(lastDropHeight - 600, FULL_EXTENSION_POS, 0));
                if (slides.isAtTargetPosition()) {
                    if (gamepad2.a) {
                        setActiveScoringState(ScoringState.RESETTING_STAGE_TWO);
                    }
                }
                break;

            case RESETTING_STAGE_TWO:
                slides.update(slides.getActiveResetPos());
                arm.setRetractPos();
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
                break;
            case HANGING:
                arm.setSafeRetractPos();
                arm.retract();
                claw.stopIntake();
                switch (activeHangingState) {
                    case MAX:
                        slides.update(FULL_EXTENSION_POS);
                        break;
                    case HANG:
                        slides.update(HANGING_POS);
                        break;
                    case RESET:
                        slides.update(slides.getActiveResetPos());
                        break;
                }

                if (gamepad2.dpad_up) {
                    activeHangingState = HangingState.MAX;
                } else if (gamepad2.dpad_down) {
                    activeHangingState = HangingState.RESET;
                } else if (gamepad2.dpad_left) {
                    activeHangingState = HangingState.HANG;
                }
                break;

            case REZEROING_1:
                arm.retract();
                slides.setPower(.35);
                if (slides.isSlidesOverCurrent()) {
                    slides.stop();
                    reZeroPos = slides.getLastPosition();
                    activeScoringState = ScoringState.REZEROING_2;
                }
                break;

            case REZEROING_2:
                slides.update(reZeroPos - 155);
                if (slides.isAtTargetPosition()) {
                    slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    activeScoringState = ScoringState.PICKING_UP;
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