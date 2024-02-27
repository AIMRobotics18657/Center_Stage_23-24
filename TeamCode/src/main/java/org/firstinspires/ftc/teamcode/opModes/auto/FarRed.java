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

@Autonomous(name = "FarRed", group = "AAA_COMP", preselectTeleOp="CompTeleOp")
public final class FarRed extends LinearOpMode {
    Robot robot = new Robot(true, FinalsAutoConstants.START_FAR_RED_POSE, true);
    int randomization;

    boolean isAtPixelPrep = false;
    boolean hasPixelDropped = false;
    boolean isAtPark = false;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("REGION: ", robot.drivebase.camera.whichRegion());
            telemetry.update();
        }

        robot.drivebase.camera.stopStreaming();

        while (opModeIsActive() && !isStopRequested()) {
            randomization = robot.drivebase.camera.whichRegion();
            Action driveToPurpleDrop;

            Pose2d purpleDropSpot;
            Vector2d yellowDropSpot;
            if (randomization == 1) {
                purpleDropSpot = FinalsAutoConstants.P_DROP_RED_1_A;
                yellowDropSpot = FinalsAutoConstants.Y_DROP_RED_1;

                driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                        .splineToConstantHeading(FinalsAutoConstants.P_DROP_RED_1_A.position, FinalsAutoConstants.P_DROP_RED_1_A_TANGENT)
                        .build();
            } else if (randomization == 2) {
                purpleDropSpot = FinalsAutoConstants.P_DROP_RED_2_B;
                yellowDropSpot = FinalsAutoConstants.Y_DROP_RED_2;

                driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                        .splineToConstantHeading(FinalsAutoConstants.P_DROP_RED_2_B.position, FinalsAutoConstants.P_DROP_RED_2_B_TANGENT)
                        .build();
            } else {
                purpleDropSpot = FinalsAutoConstants.P_DROP_RED_3_B;
                yellowDropSpot = FinalsAutoConstants.Y_DROP_RED_3;

                driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                        .strafeTo(FinalsAutoConstants.P_DROP_RED_3_A)
                        .splineToSplineHeading(FinalsAutoConstants.P_DROP_RED_3_B, FinalsAutoConstants.P_DROP_RED_3_B_TANGENT)
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
                                        robot.pixelManipulator.slides.update(PIDSlides.SAFE_RESET_POS);
                                        return isAtPixelPrep;
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
                                            },
                                            driveToPixelBoard,
                                            (telemetryPacket) -> { // End parallel action
                                                isAtPixelPrep = true;
                                                return false;
                                            }
                                        )
                                    ),
                            new ParallelAction(
                                    (telemetryPacket) -> { // Lift Slides
                                        robot.pixelManipulator.slides.update(PIDSlides.MIN_EXTENSION_POS);
                                        return hasPixelDropped;
                                    },
                                    (telemetryPacket) -> { // Extend Arm
                                        robot.pixelManipulator.arm.autoExtend();
                                        return false;
                                    },
                                    new SequentialAction(
                                            driveToYellowDrop,
                                            (telemetryPacket) -> { // Drop Yellow
                                                robot.pixelManipulator.claw.releaseServo(robot.pixelManipulator.claw.rightClamp);
                                                return false;
                                            },
                                            new SleepAction(1.0),
                                            (telemetryPacket) -> { // End parallel action
                                                hasPixelDropped = true;
                                                return false;
                                            }
                                    )
                            ),
                            new SequentialAction(
                                    (telemetryPacket) -> { // Retract Arm and Clamp Releases
                                        robot.pixelManipulator.arm.retract();
                                        robot.pixelManipulator.claw.clampServo(robot.pixelManipulator.claw.leftClamp);
                                        robot.pixelManipulator.claw.clampServo(robot.pixelManipulator.claw.rightClamp);
                                        return false;
                                    },
                                    new SleepAction(1.0),
                                    new ParallelAction(
                                            (telemetryPacket) -> { // Retract Slides Almost
                                                robot.pixelManipulator.slides.update(PIDSlides.SAFE_RESET_POS);
                                                return isAtPark;
                                            },
                                            new SequentialAction(
                                                    driveToPark,
                                                    (telemetryPacket) -> { // End Auto
                                                        isAtPark = true;
                                                        return false;
                                                    }
                                            )
                                    ),
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
