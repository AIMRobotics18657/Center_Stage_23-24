package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="SEXPO", group="SEXPO")

public class SEXPO extends OpMode {
    public MecanumHardware mecanumHardware = new MecanumHardware();

    public HardwareMap hwMap = null;

    @Override
    public void init() {
        // Save reference to Hardware map
        mecanumHardware.init(hardwareMap);
        mecanumHardware.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mecanumHardware.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mecanumHardware.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mecanumHardware.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
    }


    @Override
    public void loop() {

        double frontLeftPower = 0;
        double backLeftPower = 0;
        double frontRightPower = 0;
        double backRightPower = 0;

        if (gamepad1.a) {
            frontLeftPower = .6;
            backLeftPower = .6;
            frontRightPower = .6;
            backRightPower = .6;
        }

        mecanumHardware.leftFront.setPower(frontLeftPower);
        mecanumHardware.leftRear.setPower(backLeftPower);
        mecanumHardware.rightFront.setPower(frontRightPower);
        mecanumHardware.rightRear.setPower(backRightPower);
    }

    @Override
    public void stop() {
    }

}