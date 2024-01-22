package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.PAL;
import org.firstinspires.ftc.teamcode.subsystems.PIDSlides;
import org.firstinspires.ftc.teamcode.subsystems.settings.ConfigInfo;
import org.firstinspires.ftc.teamcode.subsystems.settings.GamepadSettings;

@TeleOp(name="SystemsCheck", group="AAA_COMPETITION")
public class SystemsCheck extends OpMode {

    private final String[] deviceNames = ConfigInfo.getAllDeviceNames();
    private SystemCheckStage currentStage = SystemCheckStage.CONNECTION_CHECK;

    // Enum representing the different stages
    private enum SystemCheckStage {
        CONNECTION_CHECK,
        INDIVIDUAL_CHECK
    }
    Drivebase drivebase = new Drivebase(new Pose2d(0, 0, 0));
    Odometry odometry = new Odometry();
    Camera camera = new Camera(false);
    Claw claw = new Claw();
    Arm arm = new Arm();
    PIDSlides slides = new PIDSlides();
    PAL pal = new PAL();

    enum TestingState {
        DRIVEBASE, ODOMETRY, CLAW, ARM, SLIDES, PAL
    }

    TestingState activeTestingState = TestingState.DRIVEBASE;

    @Override
    public void init() {
        drivebase.init(hardwareMap);
        odometry.init(hardwareMap);
        claw.init(hardwareMap);
        arm.init(hardwareMap);
        slides.init(hardwareMap);
        pal.init(hardwareMap);
    }

    @Override
    public void loop() {
        switch (currentStage) {
            case CONNECTION_CHECK:
                // Stage 0: Check if all devices are connected
                boolean allDevicesConnected = true;
                for (String deviceName : deviceNames) {
                    HardwareDevice device = hardwareMap.get(deviceName);
                    if (device == null) {
                        allDevicesConnected = false;
                        telemetry.addData(deviceName, "Not connected");
                    }
                }

                if (allDevicesConnected) {
                    telemetry.addData("Stage 0: Connection Check", "All Devices Connected");
                    telemetry.addData("Press lb & rb to Start Individual System Checks", "");
                    // Check for button press to activate next stage
                    if (gamepad1.right_bumper && gamepad1.left_bumper) {
                        currentStage = SystemCheckStage.INDIVIDUAL_CHECK; // Transition to next stage
                    }
                } else {
                    telemetry.addData("Stage 0: Connection Check", "Not all devices connected");
                    telemetry.addData("Bypass with lt and rt", "");
                    if (gamepad1.left_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE && gamepad1.right_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE) {
                        currentStage = SystemCheckStage.INDIVIDUAL_CHECK; // Transition to next stage
                    }
                }
                break;

            case INDIVIDUAL_CHECK:
                // Stage 1: Individual system checks
                telemetry.addData("Stage 1: Individual System Checks", "Performing Individual System Checks");
                switch (activeTestingState) {
                    case DRIVEBASE:
                        drivebaseTest();
                        break;
                    case ODOMETRY:
                        odoTest();
                        break;
                    case CLAW:
                        clawTest();
                        break;
                    case ARM:
                        armTest();
                        break;
                    case SLIDES:
                        slidesTest();
                        break;
                    case PAL:
                        palTest();
                        break;
                }
                telemetry.addData("Current Testing State", activeTestingState);
                telemetry.update();
        }
    }


    public void drivebaseTest() {
        drivebase.systemsCheck(gamepad1, telemetry);
        if (gamepad1.back && gamepad2.back) {
            activeTestingState = TestingState.ODOMETRY;
        }
    }

    public void odoTest() {
        odometry.systemsCheck(gamepad1, telemetry);
        if (gamepad1.start && gamepad2.start) {
            activeTestingState = TestingState.CLAW;
        }
    }

    public void clawTest() {
        claw.systemsCheck(gamepad1, telemetry);
        if (gamepad1.back && gamepad2.back) {
            activeTestingState = TestingState.ARM;
        }
    }

    public void armTest() {
        arm.systemsCheck(gamepad1, telemetry);
        if (gamepad1.start && gamepad2.start) {
            activeTestingState = TestingState.SLIDES;
        }
    }

    public void slidesTest() {
        slides.systemsCheck(gamepad1, telemetry);
        if (gamepad1.back && gamepad2.back) {
            activeTestingState = TestingState.PAL;
        }
    }

    public void palTest() {
        pal.systemsCheck(gamepad1, telemetry);
        if (gamepad1.start && gamepad2.start) {
            activeTestingState = TestingState.DRIVEBASE;
        }
    }
}
