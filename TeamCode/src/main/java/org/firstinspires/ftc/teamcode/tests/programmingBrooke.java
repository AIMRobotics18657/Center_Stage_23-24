package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="ProgrammingBrooke", group="FAF")
public class programmingBrooke extends LinearOpMode {

    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightRear = null;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "fld");
        rightFront = hardwareMap.get(DcMotorEx.class, "frd");
        leftRear = hardwareMap.get(DcMotorEx.class, "bld");
        rightRear = hardwareMap.get(DcMotorEx.class, "brd");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        run(0.5,0.5,1000);
        turn(-0.5,0.5,570);
        run (0.5,0.5,1000);
        turn(-0.5,0.5,570);
        run (0.5,0.5,1000);
        turn(-0.5,0.5,570);
        run (0.5,0.5,1000);
        turn(-0.5,0.5,570);






    }

    void run(double leftPower,double rightPower, long time) {
        leftFront.setPower(leftPower);
        rightFront.setPower(rightPower);
        leftRear.setPower(leftPower);
        rightRear.setPower(rightPower);
        sleep(time);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
    void turn(double rightPower,double leftPower,long time){
        leftFront.setPower(leftPower);
        leftRear.setPower(leftPower);
        rightRear.setPower(rightPower);
        rightFront.setPower(rightPower);
        sleep(time);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

}
