package org.firstinspires.ftc.teamcode.opModes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name="CameraTest", group="Tests")
public class CameraTestTeleOp extends Autonomous {

    Robot robot;


    @Override
    public void init() {
        robot = new Robot(true);
        robot.init(hardwareMap);
    }

    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        robot.loop(gamepad1, gamepad2);
        telemetry.addData("Camera State", robot.drivebase.getGameRandomization());


        // Display camera-related telemetry
//        telemetry.addData("Camera State", robot.getCameraState());
//        telemetry.addData("Number of Detections", robot.getDetections().size());

        // Update telemetry
        telemetry.update();
    }
}
