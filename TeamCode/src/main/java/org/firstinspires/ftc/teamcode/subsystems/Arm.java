package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.settings.ConfigInfo;
import org.firstinspires.ftc.teamcode.util.Mechanism;
import org.firstinspires.ftc.teamcode.util.ServoUtil;


public class Arm extends Mechanism {
    public Servo leftArm;
    public Servo rightArm;
    private static final double EXTENDED_POSITION = 0.67;
    private static final double RETRACTED_POSITION = 1;
    private static final double FULL_RETRACTED = 0.97;

    private static final double CLOSE_THRESHOLD = 0.01;

    private static double activeRetractPos = RETRACTED_POSITION;

    public boolean isExtended = false;
    public boolean isRetracted = false;

    @Override
    public void init(HardwareMap hwMap) {
        leftArm = hwMap.get(Servo.class, ConfigInfo.leftArm.getDeviceName());
        rightArm = hwMap.get(Servo.class, ConfigInfo.rightArm.getDeviceName());
        rightArm.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop(Gamepad gamepad) {
        isExtended = ServoUtil.isClose(leftArm, EXTENDED_POSITION, CLOSE_THRESHOLD) && ServoUtil.isClose(rightArm, EXTENDED_POSITION, CLOSE_THRESHOLD);
        isRetracted = ServoUtil.isClose(leftArm, activeRetractPos, CLOSE_THRESHOLD) && ServoUtil.isClose(rightArm, activeRetractPos, CLOSE_THRESHOLD);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Left Arm Position:", leftArm.getPosition());
        telemetry.addData("Right Arm Position:", rightArm.getPosition());
        telemetry.addData("isExtended: ", isExtended);
        telemetry.addData("isRetracted: ", isRetracted);
    }

    public void extend() {
        leftArm.setPosition(EXTENDED_POSITION);
        rightArm.setPosition(EXTENDED_POSITION);
    }

    public void retract() {
        leftArm.setPosition(activeRetractPos);
        rightArm.setPosition(activeRetractPos);
    }

    public void setSafeRetractPos() {
        activeRetractPos = FULL_RETRACTED;
    }

    public void setRetractPos() {
        activeRetractPos = RETRACTED_POSITION;
    }

    @Override
    public void systemsCheck(Gamepad gamepad, Telemetry telemetry) {
        if (gamepad.a) {
            extend();
        } else if (gamepad.b) {
            retract();
        } else if (gamepad.dpad_up) {
            ServoUtil.increment(leftArm, rightArm, 0.01);
        } else if (gamepad.dpad_down) {
            ServoUtil.increment(leftArm, rightArm, -0.01);
        }

        telemetry.addData("Left Arm Position:", leftArm.getPosition());
        telemetry.addData("Right Arm Position:", rightArm.getPosition());
        telemetry.addData("isExtended: ", isExtended);
        telemetry.addData("isRetracted: ", isRetracted);
    }
}
