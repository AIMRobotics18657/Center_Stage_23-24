package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.settings.ConfigInfo;
import org.firstinspires.ftc.teamcode.util.Mechanism;
import org.firstinspires.ftc.teamcode.util.ServoUtil;

public class PAL extends Mechanism {
    private Servo releaseServo;
    boolean enabled = false;

    private static final double RELEASE_POSITION = 0.4;
    private static final double CLAMP_POSITION = 0.69;
    @Override
    public void init(HardwareMap hwMap) {
        releaseServo = hwMap.get(Servo.class, ConfigInfo.releaseServo.getDeviceName());
        releaseServo.setPosition(CLAMP_POSITION);
        disable();
    }

    @Override
    public void loop(Gamepad gamepad) {

        boolean shooterEnabled = (gamepad.dpad_up && enabled) || (gamepad.dpad_up && gamepad.y);
        if (shooterEnabled) {
            releaseServo.setPosition(RELEASE_POSITION);
        } else {
            releaseServo.setPosition(CLAMP_POSITION);
        }
    }

    @Override
    public void systemsCheck(Gamepad gamepad, Telemetry telemetry) {
        if (gamepad.dpad_up) {
            releaseServo.setPosition(RELEASE_POSITION);
        } else if (gamepad.dpad_down) {
            releaseServo.setPosition(CLAMP_POSITION);
        } else if (gamepad.dpad_left) {
            ServoUtil.increment(releaseServo, 0.005);
        } else if (gamepad.dpad_right) {
            ServoUtil.increment(releaseServo, -0.005);
        }
        telemetry.addData("Release Servo Position: ", releaseServo.getPosition());
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }
}
