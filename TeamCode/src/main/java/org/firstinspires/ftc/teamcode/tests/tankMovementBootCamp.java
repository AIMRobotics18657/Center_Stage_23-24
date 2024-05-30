package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="tankMovementBootCamp", group="AAAAA")
public class tankMovementBootCamp extends OpMode {

    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightRear = null;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "fld");
        rightFront = hardwareMap.get(DcMotorEx.class, "frd");
        leftRear = hardwareMap.get(DcMotorEx.class, "bld");
        rightRear = hardwareMap.get(DcMotorEx.class, "brd");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        leftFront.setPower(-gamepad1.left_stick_y);
        leftRear.setPower(-gamepad1.left_stick_y);
        rightFront.setPower(-gamepad1.right_stick_y);
        rightRear.setPower(-gamepad1.right_stick_y);

    }

}
