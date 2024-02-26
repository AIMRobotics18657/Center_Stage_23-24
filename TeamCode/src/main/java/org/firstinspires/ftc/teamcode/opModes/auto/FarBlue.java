package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "FarBlue", group = "AAA_COMP", preselectTeleOp="CompTeleOp")
public final class FarBlue extends LinearOpMode {
    Robot robot = new Robot(true, FinalsAutoConstants.START_FAR_BLUE_POSE, true);
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
//            randomization = robot.drivebase.getGameRandomization();
            randomization = 3;
            Action driveToPurpleDrop;

            Pose2d purpleDropSpot;
            Vector2d yellowDropSpot;
            if (randomization == 1) {
                purpleDropSpot = FinalsAutoConstants.P_DROP_BLUE_1_B;
                yellowDropSpot = FinalsAutoConstants.Y_DROP_RED_1;

                driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                        .strafeTo(FinalsAutoConstants.P_DROP_BLUE_1_A)
                        .splineToSplineHeading(FinalsAutoConstants.P_DROP_BLUE_1_B, FinalsAutoConstants.P_DROP_BLUE_1_B_TANGENT)
                        .build();
            } else if (randomization == 2) {
                purpleDropSpot = FinalsAutoConstants.P_DROP_BLUE_2_B;
                yellowDropSpot = FinalsAutoConstants.Y_DROP_RED_2;

                driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                        .splineToConstantHeading(FinalsAutoConstants.P_DROP_BLUE_2_B.position, FinalsAutoConstants.P_DROP_BLUE_2_B_TANGENT)
                        .build();
            } else {
                purpleDropSpot = FinalsAutoConstants.P_DROP_BLUE_3_A;
                yellowDropSpot = FinalsAutoConstants.Y_DROP_RED_3;

                driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                        .splineToConstantHeading(FinalsAutoConstants.P_DROP_BLUE_3_A.position, FinalsAutoConstants.P_DROP_BLUE_3_A_TANGENT)
                        .build();
            }

            Action driveToPixelBoard = robot.drivebase.drive.actionBuilder(purpleDropSpot)
                    .strafeTo(FinalsAutoConstants.TRUSS_PASS_RED_A)
                    .strafeToSplineHeading(FinalsAutoConstants.TRUSS_PASS_RED_B.position, FinalsAutoConstants.TRUSS_PASS_RED_B.heading)
                    .strafeTo(FinalsAutoConstants.SPLINE_PRE_RED)
                    .splineToConstantHeading(FinalsAutoConstants.SPLINE_POST_RED, FinalsAutoConstants.SPLINE_POST_RED_TANGENT)
                    .build();

            Action driveToYellowDrop = robot.drivebase.drive.actionBuilder(new Pose2d(FinalsAutoConstants.SPLINE_POST_RED, FinalsAutoConstants.SPLINE_POST_RED_HEADING))
                    .strafeTo(yellowDropSpot)
                    .build();

            Action driveToPark = robot.drivebase.drive.actionBuilder(new Pose2d(yellowDropSpot, FinalsAutoConstants.SPLINE_POST_RED_HEADING))
                    .strafeTo(FinalsAutoConstants.SAFE_PARK_TRANSITION_RED)
                    .strafeToLinearHeading(FinalsAutoConstants.PARK_RED.position, FinalsAutoConstants.PARK_RED.heading)
                    .build();

            //DRIVE TO DROP PIXEL
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    (telemetryPacket) -> { // Slight lift in slides
                                        robot.pixelManipulator.arm.retract();
                                        robot.pixelManipulator.slides.update(-80);
                                        return true;
                                    },
                                    new SequentialAction(
                                            driveToPurpleDrop,
                                            (telemetryPacket) -> { // Drop Purple
                                                robot.pixelManipulator.claw.outtake();
                                                return false;
                                            },
                                            new SleepAction(0.2),
                                            (telemetryPacket) -> { // Drop Purple
                                                robot.pixelManipulator.claw.stopIntake();
                                                robot.pixelManipulator.arm.autoExtend();
                                                return false;
                                            }
//                                            ,driveToPixelBoard
                                        )
                                    )
//                            ,(telemetryPacket) -> { // Lift Slides
//                                robot.pixelManipulator.slides.setPower(-0.3);
//                                robot.pixelManipulator.arm.extend();
//                                return false;
//                            },
//                            new SleepAction(slideSleepInSeconds),
//                            (telemetryPacket) -> { // Stop Slides
//                                robot.pixelManipulator.slides.setPower(0);
//                                return false;
//                            },
//                            driveToYellowDrop,
//                            (telemetryPacket) -> { // Drop Yellow
//                                robot.pixelManipulator.claw.releaseServo(robot.pixelManipulator.claw.rightClamp);
//                                return false;
//                            },
//                            new SleepAction(sleepInSeconds),
//                            driveToPark,
//                            (telemetryPacket) -> { // Retract Arm
//                                robot.pixelManipulator.arm.retract();
//                                robot.pixelManipulator.claw.clampServo(robot.pixelManipulator.claw.leftClamp);
//                                robot.pixelManipulator.claw.clampServo(robot.pixelManipulator.claw.rightClamp);
//                                return robot.pixelManipulator.arm.isRetracted;
//                            },
//                            new SleepAction(sleepInSeconds),
//                            (telemetryPacket) -> { // Retract Slides
//                                robot.pixelManipulator.slides.setPower(0.3);
//                                return false;
//                            },
//                            new SleepAction(slideSleepInSeconds)
                    )
            );
            break;
        }
    }
}
