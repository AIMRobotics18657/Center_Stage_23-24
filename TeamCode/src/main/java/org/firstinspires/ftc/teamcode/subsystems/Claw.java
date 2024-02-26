package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.settings.ConfigInfo;
import org.firstinspires.ftc.teamcode.util.Mechanism;
import org.firstinspires.ftc.teamcode.util.ServoUtil;

public class Claw extends Mechanism {

        public DcMotorEx spinningIntake;
        public Servo leftClamp;
        public Servo rightClamp;
        private static final double RELEASE_POSITION = 0.845;
        private static final double GATE_POSITION = 1;

        public boolean isLeftClamped = false;
        public boolean isLeftReleased = true;
        public boolean isRightClamped = false;
        public boolean isRightReleased = true;

        @Override
        public void init(HardwareMap hwMap) {
            spinningIntake = hwMap.get(DcMotorEx.class, ConfigInfo.intake.getDeviceName());
            leftClamp = hwMap.get(Servo.class, ConfigInfo.leftClamp.getDeviceName());
            rightClamp = hwMap.get(Servo.class, ConfigInfo.rightClamp.getDeviceName());

            spinningIntake.setDirection(DcMotorSimple.Direction.REVERSE);
            leftClamp.setDirection(Servo.Direction.REVERSE);
            clampServo(leftClamp);
            clampServo(rightClamp);
        }

        @Override
        public void loop(Gamepad gamepad) {

            isLeftClamped = ServoUtil.isClose(leftClamp, GATE_POSITION, 0.01);
            isRightClamped = ServoUtil.isClose(rightClamp, GATE_POSITION, 0.01);
            isLeftReleased = ServoUtil.isClose(leftClamp, RELEASE_POSITION, 0.01);
            isRightReleased = ServoUtil.isClose(rightClamp, RELEASE_POSITION, 0.01);
        }

        @Override
        public void telemetry(Telemetry telemetry) {
            telemetry.addData("Left Prong Position", leftClamp.getPosition());
            telemetry.addData("Right Prong Position", rightClamp.getPosition());
            telemetry.addData("isLeftClamped", isLeftClamped);
            telemetry.addData("isRightClamped", isRightClamped);
            telemetry.addData("isLeftReleased", isLeftReleased);
            telemetry.addData("isRightReleased", isRightReleased);
        }

        public void clampServo(Servo servo) {
            servo.setPosition(GATE_POSITION);
        }

        public void releaseServo(Servo servo) {
            servo.setPosition(RELEASE_POSITION);
        }

        public void intake() {
            spinningIntake.setPower(0.6);
        }

        public void outtake() {
            spinningIntake.setPower(-0.35);
        }

        public void stopIntake() {
            spinningIntake.setPower(0);
        }

        @Override
        public void systemsCheck(Gamepad gamepad, Telemetry telemetry) {
            if (gamepad.a) {
                clampServo(leftClamp);
            } else if (gamepad.b) {
                releaseServo(leftClamp);
            } else if (gamepad.x) {
                clampServo(rightClamp);
            } else if (gamepad.y) {
                releaseServo(rightClamp);
            } else if (gamepad.dpad_up) {
                ServoUtil.increment(leftClamp, rightClamp, 0.005);
            } else if (gamepad.dpad_down) {
                ServoUtil.increment(leftClamp, rightClamp, -0.005);
            }

            if (gamepad.left_trigger > 0.01){
                spinningIntake.setPower(gamepad.left_trigger);
            } else if (gamepad.right_trigger > 0.01){
                spinningIntake.setPower(-gamepad.right_trigger);
            }


            telemetry.addData("Left Prong Position", leftClamp.getPosition());
            telemetry.addData("Right Prong Position", rightClamp.getPosition());
            telemetry.addData("isLeftClamped", isLeftClamped);
            telemetry.addData("isRightClamped", isRightClamped);
            telemetry.addData("isLeftReleased", isLeftReleased);
            telemetry.addData("isRightReleased", isRightReleased);
        }
    }


