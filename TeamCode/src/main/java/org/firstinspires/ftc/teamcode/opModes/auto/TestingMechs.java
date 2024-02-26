package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "AA_TestingMechs", group = "Tests", preselectTeleOp="CompTeleOp")
public final class TestingMechs extends LinearOpMode {
    Robot robot = new Robot(true, FinalsAutoConstants.START_FAR_RED_POSE, true);
    int randomization;

    private static final int sleepInSeconds = 1;
    private static final double slideSleepInSeconds = 1.3;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();
        while (opModeInInit()) {
            telemetry.addData("Randomization", robot.drivebase.getGameRandomization());
            telemetry.update();
        }
        while (opModeIsActive()) {

            //DRIVE TO DROP PIXEL
            Actions.runBlocking(
                    new SequentialAction(
                            (telemetryPacket) -> { // Extend Arm
                                robot.pixelManipulator.arm.autoExtend();
                                return false;
                            },
                            new SleepAction(sleepInSeconds),
                            (telemetryPacket) -> { // Drop Purple
                                robot.pixelManipulator.claw.outtake();
                                return false;
                            },
                            new SleepAction(0.7),
                            (telemetryPacket) -> { // Stop Intake
                                robot.pixelManipulator.claw.stopIntake();
                                return false;
                            }
                    )
            );
            break;
        }
    }
}
