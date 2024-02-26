package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Camera;

@Autonomous(name="CameraTest", group="Tests")
public class CameraTestAutoOp extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(true, new Pose2d(0, 0, 0), true);
        robot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            robot.drivebase.camera.telemetryTfod(telemetry);
            telemetry.addData("Game Randomization", robot.drivebase.getGameRandomization());

            telemetry.update();
        }
    }
}
