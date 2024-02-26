package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.settings.GamepadSettings;
import org.firstinspires.ftc.teamcode.util.Mechanism;

public class Robot extends Mechanism {

    public PixelManipulator pixelManipulator;
    public Drivebase drivebase;
    private Hubs hubs;
    MatchTimer timer = new MatchTimer();

    PAL pal;

    private Pose2d startingPose;

    private boolean resetIMU;

    boolean isRedAlliance;
    boolean isEndGame = false;
    boolean isDrivebaseSpeeding = false;
    boolean hasEndgameRumbled = false;
    private static final int ENDGAME_TIMESTAMP = 90;

    public Robot(boolean isRedAlliance, Pose2d startingPose, boolean resetIMU) {
        this.isRedAlliance = isRedAlliance;
        this.startingPose = startingPose;
        this.resetIMU = resetIMU;
    }
    @Override
    public void init(HardwareMap hwMap) {
        pixelManipulator = new PixelManipulator();
        drivebase = new Drivebase(isRedAlliance, startingPose, resetIMU);
        pal = new PAL();
        hubs = new Hubs();

        pixelManipulator.init(hwMap);
        drivebase.init(hwMap);
        pal.init(hwMap);
        hubs.init(hwMap);
        timer.init(hwMap);
    }

    @Override
    public void loop(Gamepad gamepad, Gamepad gamepad2) {
        pixelManipulator.loop(gamepad, gamepad2);
        drivebase.loop(gamepad);
        timer.loop(gamepad);

        isEndGame = timer.getTime() > ENDGAME_TIMESTAMP;
        isDrivebaseSpeeding = drivebase.isDriveSpeeding(gamepad);
        if (isEndGame) {
            if (!hasEndgameRumbled) {
                gamepad.runRumbleEffect(GamepadSettings.endgameEffect);
                gamepad2.runRumbleEffect(GamepadSettings.endgameEffect);
                hasEndgameRumbled = true;
            }
            pal.loop(gamepad);
            pixelManipulator.setHangModeEnabled(true);
        }
        if (isDrivebaseSpeeding) {
            pixelManipulator.slides.setSafeResetPos();
        } else {
            pixelManipulator.slides.setResetPos();
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
