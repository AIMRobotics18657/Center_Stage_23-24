package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opModes.auto.FinalsAutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name="BlueTeleOp", group="Tests")
public class BlueTeleOp extends OpMode {

    Robot robot;

    @Override
    public void init() {
        robot = new Robot(false, FinalsAutoConstants.PARK_BLUE_AWAY, false);
        robot.init(hardwareMap);
    }

    @Override
    public void start() {
        robot.start();
        robot.drivebase.camera.stopStreaming();
    }

    @Override
    public void loop() {
        robot.loop(gamepad1, gamepad2);
        robot.telemetry(telemetry);
        telemetry.update();
    }
}