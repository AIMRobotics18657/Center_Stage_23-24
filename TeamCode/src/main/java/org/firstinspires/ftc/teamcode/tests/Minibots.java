package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Minibots", group="FAF")
public class Minibots extends OpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void init() {
        // Initialize motors
        leftMotor = hardwareMap.dcMotor.get("left");  // Replace "left_motor" with the actual name in your robot configuration
        rightMotor = hardwareMap.dcMotor.get("right");  // Replace "right_motor" with the actual name in your robot configuration

        // Set motor directions
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // TODO Adjust if needed
        rightMotor.setDirection(DcMotor.Direction.REVERSE); // TODO Adjust if needed
    }

    @Override
    public void loop() {
        // Tank drive control
        double leftPower = -gamepad1.left_stick_y;  // Adjust if needed
        double rightPower = gamepad1.right_stick_y;  // Adjust if needed

        // Set motor powers
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }
}
