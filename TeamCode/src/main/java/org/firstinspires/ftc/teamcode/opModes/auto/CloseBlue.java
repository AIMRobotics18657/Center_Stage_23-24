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

@Autonomous(name = "FarBlue", group = "AAA_COMP", preselectTeleOp="CompTeleOp")
public final class CloseBlue extends LinearOpMode {
    Robot robot = new Robot(false, FinalsAutoConstants.START_NEAR_BLUE_POSE, true);
    int randomization;

    boolean isAtPixelPrep = true;
    boolean hasPixelDropped = true;
    boolean isAtPark = true;

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
            if (randomization == 3) {
                purpleDropSpot = FinalsAutoConstants.P_DROP_BLUE_3_B_CLOSE;
                yellowDropSpot = FinalsAutoConstants.Y_DROP_BLUE_3;

                driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                        .strafeTo(FinalsAutoConstants.P_DROP_BLUE_3_A_CLOSE)
                        .splineToSplineHeading(FinalsAutoConstants.P_DROP_BLUE_3_B_CLOSE, FinalsAutoConstants.P_DROP_BLUE_3_B_TANGENT_CLOSE)
                        .build();
            } else if (randomization == 2) {
                purpleDropSpot = FinalsAutoConstants.P_DROP_BLUE_2_A_CLOSE;
                yellowDropSpot = FinalsAutoConstants.Y_DROP_BLUE_2;

                driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                        .splineToConstantHeading(FinalsAutoConstants.P_DROP_BLUE_2_A_CLOSE.position, FinalsAutoConstants.P_DROP_BLUE_2_A_TANGENT_CLOSE)
                        .build();
            } else {
                purpleDropSpot = FinalsAutoConstants.P_DROP_BLUE_1_A_CLOSE;
                yellowDropSpot = FinalsAutoConstants.Y_DROP_BLUE_1;

                driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                        .splineToConstantHeading(FinalsAutoConstants.P_DROP_BLUE_1_A_CLOSE.position, FinalsAutoConstants.P_DROP_BLUE_1_A_TANGENT_CLOSE)
                        .build();
            }

            Action driveToPixelBoard = robot.drivebase.drive.actionBuilder(purpleDropSpot)
                    .splineToSplineHeading(new Pose2d(FinalsAutoConstants.SPLINE_POST_BLUE, FinalsAutoConstants.PIXEL_BOARD_HEADING), FinalsAutoConstants.SPLINE_POST_BLUE_TANGENT)
                    .build();

            Action driveToYellowDrop = robot.drivebase.drive.actionBuilder(new Pose2d(FinalsAutoConstants.SPLINE_POST_BLUE, FinalsAutoConstants.PIXEL_BOARD_HEADING))
                    .strafeTo(yellowDropSpot)
                    .build();

            Action driveToPark = robot.drivebase.drive.actionBuilder(new Pose2d(yellowDropSpot, FinalsAutoConstants.PIXEL_BOARD_HEADING))
                    .strafeTo(FinalsAutoConstants.SAFE_PARK_TRANSITION_BLUE)
                    .strafeToLinearHeading(FinalsAutoConstants.PARK_BLUE.position, FinalsAutoConstants.PARK_BLUE.heading)
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
                                                robot.pixelManipulator.arm.autoExtend();
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
                                                        isAtPark = false;
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
