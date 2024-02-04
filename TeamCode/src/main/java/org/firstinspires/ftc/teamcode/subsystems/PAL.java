package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.settings.ConfigInfo;
import org.firstinspires.ftc.teamcode.util.Mechanism;
import org.firstinspires.ftc.teamcode.util.ServoUtil;

public class PAL extends Mechanism {

    private DcMotorEx leftFly;
    private DcMotorEx rightFly;
    private Servo releaseServo;

    private static final double RELEASE_POSITION = 0.3;
    private static final double RETRACT_POSITION = 0.67;
    @Override
    public void init(HardwareMap hwMap) {
        leftFly = hwMap.get(DcMotorEx.class, ConfigInfo.leftFly.getDeviceName());
        rightFly = hwMap.get(DcMotorEx.class, ConfigInfo.rightFly.getDeviceName());
        releaseServo = hwMap.get(Servo.class, ConfigInfo.releaseServo.getDeviceName());

        leftFly.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFly.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFly.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        releaseServo.setPosition(RETRACT_POSITION);
    }

    @Override
    public void loop(Gamepad gamepad) {
        leftFly.setPower(gamepad.left_trigger);
        rightFly.setPower(gamepad.left_trigger);
        if (gamepad.dpad_up) {
            releaseServo.setPosition(RELEASE_POSITION);
        }
    }

    @Override
    public void systemsCheck(Gamepad gamepad, Telemetry telemetry) {
        leftFly.setPower(gamepad.right_trigger);
        rightFly.setPower(gamepad.right_trigger);
        if (gamepad.dpad_up) {
            releaseServo.setPosition(RELEASE_POSITION);
        } else if (gamepad.dpad_down) {
            releaseServo.setPosition(RETRACT_POSITION);
        } else if (gamepad.dpad_left) {
            ServoUtil.increment(releaseServo, 0.01);
        } else if (gamepad.dpad_right) {
            ServoUtil.increment(releaseServo, -0.01);
        }
        telemetry.addData("Left Flywheel Power: ", leftFly.getPower());
        telemetry.addData("Right Flywheel Power: ", rightFly.getPower());
        telemetry.addData("Release Servo Position: ", releaseServo.getPosition());
    }
}
