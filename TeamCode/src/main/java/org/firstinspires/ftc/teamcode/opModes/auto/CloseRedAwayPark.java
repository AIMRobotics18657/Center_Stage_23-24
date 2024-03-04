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

import org.firstinspires.ftc.teamcode.subsystems.PIDSlides;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "CloseRedAwayPark", group = "AAA_COMP", preselectTeleOp="RedTeleOp")
public final class CloseRedAwayPark extends LinearOpMode {
    Robot robot = new Robot(true, FinalsAutoConstants.START_NEAR_RED_POSE, true);
    int randomization;

    boolean isAtPixelPrep = true;
    boolean hasPixelDropped = true;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("REGION: ", robot.drivebase.camera.whichRegion());
            telemetry.update();
        }

        robot.drivebase.camera.stopStreaming();

        while (opModeIsActive()) {
            randomization = robot.drivebase.camera.whichRegion();
            Action driveToPurpleDrop;

            Pose2d purpleDropSpot;
            Vector2d yellowDropSpot;
            if (randomization == 1) {
                purpleDropSpot = FinalsAutoConstants.P_DROP_RED_1_B_CLOSE;
                yellowDropSpot = FinalsAutoConstants.Y_DROP_RED_1_CLOSE;

                driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                        .strafeTo(FinalsAutoConstants.P_DROP_RED_1_A_CLOSE)
                        .splineToSplineHeading(FinalsAutoConstants.P_DROP_RED_1_B_CLOSE, FinalsAutoConstants.P_DROP_RED_1_B_TANGENT_CLOSE)
                        .build();
            } else if (randomization == 2) {
                purpleDropSpot = FinalsAutoConstants.P_DROP_RED_2_A_CLOSE;
                yellowDropSpot = FinalsAutoConstants.Y_DROP_RED_2_CLOSE;

                driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                        .splineToConstantHeading(FinalsAutoConstants.P_DROP_RED_2_A_CLOSE.position, FinalsAutoConstants.P_DROP_RED_2_A_TANGENT_CLOSE)
                        .build();
            } else {
                purpleDropSpot = FinalsAutoConstants.P_DROP_RED_3_A_CLOSE;
                yellowDropSpot = FinalsAutoConstants.Y_DROP_RED_3_CLOSE;

                driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                        .splineToConstantHeading(FinalsAutoConstants.P_DROP_RED_3_A_CLOSE.position, FinalsAutoConstants.P_DROP_RED_3_A_TANGENT_CLOSE)
                        .build();
            }

            Action driveToPixelBoard = robot.drivebase.drive.actionBuilder(purpleDropSpot)
                    .strafeTo(FinalsAutoConstants.PREP_CLOSE_RED_A)
                    .strafeToSplineHeading(FinalsAutoConstants.PREP_CLOSE_RED_B.position, FinalsAutoConstants.PREP_CLOSE_RED_B.heading)
                    .strafeTo(FinalsAutoConstants.SPLINE_PRE_RED)
                    .splineToConstantHeading(FinalsAutoConstants.SPLINE_POST_RED, FinalsAutoConstants.SPLINE_POST_RED_TANGENT)
                    .build();

            Action driveToYellowDrop = robot.drivebase.drive.actionBuilder(new Pose2d(FinalsAutoConstants.SPLINE_POST_RED, FinalsAutoConstants.PIXEL_BOARD_HEADING))
                    .strafeTo(yellowDropSpot)
                    .build();

            Action driveToPark = robot.drivebase.drive.actionBuilder(new Pose2d(yellowDropSpot, FinalsAutoConstants.PIXEL_BOARD_HEADING))
                    .strafeTo(FinalsAutoConstants.SAFE_PARK_TRANSITION_RED)
                    .strafeToLinearHeading(FinalsAutoConstants.PARK_RED_AWAY.position, FinalsAutoConstants.PARK_RED_AWAY.heading)
                    .build();

            //DRIVE TO DROP PIXEL
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    (telemetryPacket) -> { // Slight lift in slides
                                        robot.pixelManipulator.arm.autoExtend();
                                        robot.pixelManipulator.slides.update(PIDSlides.AUTO_RESET_POS);
                                        return isAtPixelPrep;
                                    },
                                    new SequentialAction(
                                            driveToPurpleDrop,
                                            (telemetryPacket) -> { // Drop Purple
                                                robot.pixelManipulator.claw.spinVelo(-250);
                                                return false;
                                            },
                                            new SleepAction(0.65),
                                            (telemetryPacket) -> { // Stop Intake
                                                robot.pixelManipulator.claw.stopIntake();
                                                robot.pixelManipulator.arm.setSafeRetractPos();
                                                robot.pixelManipulator.arm.autoRetract();
                                                return false;
                                            },
                                            driveToPixelBoard,
                                            (telemetryPacket) -> { // End parallel action
                                                isAtPixelPrep = false;
                                                return false;
                                            }
                                    )
                            ),
                            new ParallelAction(
                                    (telemetryPacket) -> { // Lift Slides
                                        robot.pixelManipulator.slides.update(PIDSlides.MIN_EXTENSION_POS);
                                        return hasPixelDropped;
                                    },
                                    new SequentialAction(
                                            (telemetryPacket) -> { // Extend Arm
                                                robot.pixelManipulator.arm.extend();
                                                return false;
                                            },
                                            new SleepAction(1.0),
                                            driveToYellowDrop,
                                            (telemetryPacket) -> { // Drop Yellow
                                                robot.pixelManipulator.claw.releaseServo(robot.pixelManipulator.claw.rightClamp);
                                                return false;
                                            },
                                            new SleepAction(1.0),
                                            (telemetryPacket) -> { // End parallel action
                                                hasPixelDropped = false;
                                                return false;
                                            },
                                            (telemetryPacket) -> {
                                                robot.pixelManipulator.slides.update(PIDSlides.AUTO_LIFT_POS_FAR);
                                                return !robot.pixelManipulator.slides.isAtTargetPosition();
                                            },
                                            driveToPark
                                    )
                            ),
                            new SequentialAction(
                                    (telemetryPacket) -> { // Retract Arm and Clamp Releases
                                        robot.pixelManipulator.slides.setPower(0);
                                        robot.pixelManipulator.arm.setRetractPos();
                                        robot.pixelManipulator.arm.retract();
                                        robot.pixelManipulator.claw.clampServo(robot.pixelManipulator.claw.leftClamp);
                                        robot.pixelManipulator.claw.clampServo(robot.pixelManipulator.claw.rightClamp);
                                        return false;
                                    },
                                    new SleepAction(1.5),
                                    (telemetryPacket) -> { // Retract Slides Fully
                                        robot.pixelManipulator.slides.update(PIDSlides.RESET_POS);
                                        return !robot.pixelManipulator.slides.isAtTargetPosition();
                                    }

                            )
                    )
            );
            break;
        }
    }
}
