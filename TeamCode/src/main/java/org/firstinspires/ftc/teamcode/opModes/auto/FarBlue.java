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

@Autonomous(name = "FarBlue", group = "AAA_COMP", preselectTeleOp="BlueTeleOp")
public final class FarBlue extends LinearOpMode {
    Robot robot = new Robot(false, FinalsAutoConstants.START_FAR_BLUE_POSE, true);
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
            if (randomization == 1) {
                purpleDropSpot = FinalsAutoConstants.P_DROP_BLUE_1_B;
                yellowDropSpot = FinalsAutoConstants.Y_DROP_BLUE_1;

                driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                        .strafeTo(FinalsAutoConstants.P_DROP_BLUE_1_A)
                        .splineToSplineHeading(FinalsAutoConstants.P_DROP_BLUE_1_B, FinalsAutoConstants.P_DROP_BLUE_1_B_TANGENT)
                        .build();
            } else if (randomization == 2) {
                purpleDropSpot = FinalsAutoConstants.P_DROP_BLUE_2_A;
                yellowDropSpot = FinalsAutoConstants.Y_DROP_BLUE_2;

                driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                        .splineToConstantHeading(FinalsAutoConstants.P_DROP_BLUE_2_A.position, FinalsAutoConstants.P_DROP_BLUE_2_A_TANGENT)
                        .build();
            } else {
                purpleDropSpot = FinalsAutoConstants.P_DROP_BLUE_3_A;
                yellowDropSpot = FinalsAutoConstants.Y_DROP_BLUE_3;

                driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                        .splineToConstantHeading(FinalsAutoConstants.P_DROP_BLUE_3_A.position, FinalsAutoConstants.P_DROP_BLUE_3_A_TANGENT)
                        .build();
            }

            Action driveToPixelBoard = robot.drivebase.drive.actionBuilder(purpleDropSpot)
                    .strafeTo(FinalsAutoConstants.TRUSS_PASS_BLUE_A)
                    .strafeToSplineHeading(FinalsAutoConstants.TRUSS_PASS_BLUE_B.position, FinalsAutoConstants.TRUSS_PASS_BLUE_B.heading)
                    .strafeTo(FinalsAutoConstants.SPLINE_PRE_BLUE)
                    .splineToConstantHeading(FinalsAutoConstants.SPLINE_POST_BLUE, FinalsAutoConstants.SPLINE_POST_BLUE_TANGENT)
                    .build();

            Action driveToYellowDrop = robot.drivebase.drive.actionBuilder(new Pose2d(FinalsAutoConstants.SPLINE_POST_BLUE, FinalsAutoConstants.SPLINE_POST_BLUE_HEADING))
                    .strafeTo(yellowDropSpot)
                    .build();

            Action driveToPark = robot.drivebase.drive.actionBuilder(new Pose2d(yellowDropSpot, FinalsAutoConstants.SPLINE_POST_BLUE_HEADING))
                    .strafeTo(FinalsAutoConstants.SAFE_PARK_TRANSITION_BLUE)
//                    .strafeToLinearHeading(FinalsAutoConstants.PARK_BLUE.position, FinalsAutoConstants.PARK_BLUE.heading)
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
                                            new SleepAction(9),
                                            driveToPixelBoard,
                                            (telemetryPacket) -> { // End parallel action
                                                isAtPixelPrep = false;
                                                return false;
                                            }
                                    )
                            ),
                            new ParallelAction(
                                    (telemetryPacket) -> { // Lift Slides
                                        robot.pixelManipulator.slides.update(PIDSlides.AUTO_LIFT_POS_FAR);
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
                                            driveToPark,
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
