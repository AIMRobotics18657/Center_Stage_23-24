package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.settings.GamepadSettings;
import org.firstinspires.ftc.teamcode.util.Mechanism;

public class Robot extends Mechanism {

    public PixelManipulator pixelManipulator;
    public Drivebase drivebase;
    MatchTimer timer = new MatchTimer();

    PAL pal;

    private double startingX;
    private double startingY;
    private double startingHeading;

    boolean isRedAlliance;
    boolean isEndGame = false;
    boolean isDrivebaseSpeeding = false;
    private static final int ENDGAME_TIMESTAMP = 90;

    public Robot(boolean isRedAlliance, double startingX, double startingY, double startingHeading) {
        this.isRedAlliance = isRedAlliance;
        this.startingX = startingX;
        this.startingY = startingY;
        this.startingHeading = startingHeading;

    }
    @Override
    public void init(HardwareMap hwMap) {
        pixelManipulator = new PixelManipulator();
        drivebase = new Drivebase(new Pose2d(startingX, startingY, startingHeading), new Position(startingX, startingY, startingHeading)); // TODO: Set starting position
        pal = new PAL();

        pixelManipulator.init(hwMap);
        drivebase.init(hwMap);
        pal.init(hwMap);
        timer.init(hwMap);
    }

    @Override
    public void loop(Gamepad gamepad, Gamepad gamepad2) {
        pixelManipulator.loop(gamepad, gamepad2);
        drivebase.loop(gamepad);
        timer.loop(gamepad);

        isEndGame = timer.getTime() > ENDGAME_TIMESTAMP;
        isDrivebaseSpeeding = drivebase.isDriveSpeeding();
        if (isEndGame) {
            pal.loop(gamepad);
            pixelManipulator.setHangModeEnabled(true);
        }
        if (gamepad.right_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE) {
            pixelManipulator.slides.setSafeResetPos();
//            pixelManipulator.arm.setSafeRetractPos();
        } else {
            pixelManipulator.slides.setResetPos();
//            pixelManipulator.arm.setRetractPos();
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        pixelManipulator.telemetry(telemetry);
        drivebase.telemetry(telemetry);
        timer.telemetry(telemetry);
    }

    public void start() {
        timer.start();
    }
}
