package org.firstinspires.ftc.teamcode.tests;


// This TeleOp tests the drivebase mechanism class

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

@TeleOp(name="DrivebaseSubsystemTester", group="Tests")
public class DrivebaseSubsystemTester extends OpMode {

    Drivebase drivebase;

    @Override
    public void init() {
        drivebase = new Drivebase(true, new Pose2d(0, 0, 0), true);
        drivebase.init(hardwareMap);
    }

    @Override
    public void loop() {
        drivebase.loop(gamepad1);
        drivebase.telemetry(telemetry);
        telemetry.update();
    }
}
