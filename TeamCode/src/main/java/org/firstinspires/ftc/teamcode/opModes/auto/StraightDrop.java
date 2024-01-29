package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "StraightDrop", group = "AAAAA", preselectTeleOp="CompTeleOp")
public final class StraightDrop extends LinearOpMode {
    Robot robot = new Robot(true, 0, 0, 0);

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            robot.drivebase.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(.5, 0), 0));
            sleep(1400);
            robot.drivebase.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            sleep(1000);

            robot.pixelManipulator.slides.setPower(.02);
            robot.pixelManipulator.claw.outtake();
            sleep(10000);
            break;
        }

    }
}
