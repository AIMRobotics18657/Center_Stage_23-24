package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "BoardDropRed", group = "AAAAA")
public final class BoardDropRed extends LinearOpMode {
    Robot robot = new Robot(true, 0, 0, 0);

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        int randomization = 1;

        waitForStart();

        while (opModeIsActive()) {
            sleep(1000);
            robot.pixelManipulator.arm.extend();
            robot.drivebase.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(.5, .5), 0));
            sleep(1000);
            robot.drivebase.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            sleep(1000);
            robot.pixelManipulator.slides.setPower(-.2);
            sleep(1000);
            robot.pixelManipulator.slides.setPower(0);
            robot.drivebase.camera.checkAndSetDesiredTag(-1);
            telemetry.addData("Pose Data", robot.drivebase.camera.getDesiredTagPoseData());
            telemetry.addData("Current Detections", robot.drivebase.camera.getDetections());
            telemetry.update();
            sleep(1000);
            robot.drivebase.drive.setDrivePowers(robot.drivebase.clampSpeeds(0, 0, 0));
            sleep(5000);
            robot.drivebase.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            robot.pixelManipulator.claw.releaseServo(robot.pixelManipulator.claw.leftClamp);
            robot.pixelManipulator.claw.releaseServo(robot.pixelManipulator.claw.rightClamp);
            sleep(1000);
            robot.pixelManipulator.arm.retract();
            robot.pixelManipulator.slides.setPower(.2);
            sleep(1100);
            break;
        }

    }
}
