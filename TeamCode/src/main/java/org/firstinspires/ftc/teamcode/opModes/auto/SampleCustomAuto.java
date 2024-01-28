package org.firstinspires.ftc.teamcode.opModes.auto;

import static org.firstinspires.ftc.teamcode.subsystems.PIDSlides.PURPLE_DROP_POS;
import static org.firstinspires.ftc.teamcode.subsystems.PIDSlides.RESET_POS;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "SampleCustomAuto", group = "AAAAA")
public final class SampleCustomAuto extends LinearOpMode {
    Robot robot = new Robot(true, 12, 0, 0);

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("localizer:", robot.drivebase.localizer.getCurrentPosition().getX());
            telemetry.addData("localizer:", robot.drivebase.localizer.getCurrentPosition().getY());
            telemetry.addData("localizer:", robot.drivebase.localizer.getCurrentPosition().getHeading());
            telemetry.update();
            sleep(2000);
            robot.drivebase.driveTo(12, 12, 0);
            telemetry.addData("localizer:", robot.drivebase.localizer.getCurrentPosition().getX());
            telemetry.addData("localizer:", robot.drivebase.localizer.getCurrentPosition().getY());
            telemetry.addData("localizer:", robot.drivebase.localizer.getCurrentPosition().getHeading());
            telemetry.update();
            sleep(2000);
            robot.drivebase.driveTo(12, 12, 90);
            telemetry.addData("localizer:", robot.drivebase.localizer.getCurrentPosition().getX());
            telemetry.addData("localizer:", robot.drivebase.localizer.getCurrentPosition().getY());
            telemetry.addData("localizer:", robot.drivebase.localizer.getCurrentPosition().getHeading());
            telemetry.update();
            sleep(2000);
            robot.drivebase.driveTo(18, 18, 45);
            telemetry.addData("localizer:", robot.drivebase.localizer.getCurrentPosition().getX());
            telemetry.addData("localizer:", robot.drivebase.localizer.getCurrentPosition().getY());
            telemetry.addData("localizer:", robot.drivebase.localizer.getCurrentPosition().getHeading());
            telemetry.update();
            sleep(2000);
            break;
        }

    }
}
