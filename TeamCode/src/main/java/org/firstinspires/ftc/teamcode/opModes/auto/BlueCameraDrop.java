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

@Autonomous(name = "BlueCameraDrop", group = "Tests", preselectTeleOp="CompTeleOp")
public final class BlueCameraDrop extends LinearOpMode {
    Robot robot = new Robot(true, AutoConstants.CAMERA_DROP_START_X, AutoConstants.CAMERA_DROP_BLUE_START_Y, AutoConstants.CAMERA_DROP_BLUE_START_HEADING);
    int randomization = 1;
    Pose2d lastPurpleDropPose = new Pose2d(0, 0, 0);


    private static final int sleepInSeconds = 1;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            randomization = robot.drivebase.getGameRandomization();
            Action driveToPurpleDrop;
            if (randomization == 1) {
                lastPurpleDropPose = new Pose2d(AutoConstants.CAMERA_DROP_BLUE_LEFT_RELEASE_PURPLE_X, AutoConstants.CAMERA_DROP_BLUE_LEFT_RIGHT_RELEASE_PURPLE_Y, AutoConstants.CAMERA_DROP_BLUE_LEFT_RELEASE_PURPLE_HEADING);
                driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                        .splineToSplineHeading(new Pose2d(AutoConstants.CAMERA_DROP_BLUE_LEFT_RELEASE_PURPLE_X, AutoConstants.CAMERA_DROP_BLUE_LEFT_RIGHT_RELEASE_PURPLE_Y, AutoConstants.CAMERA_DROP_BLUE_LEFT_RELEASE_PURPLE_HEADING), AutoConstants.RELEASE_PURPLE_TANGENT)
                        .build();
            } else if (randomization == 2) {
                lastPurpleDropPose = new Pose2d(AutoConstants.CAMERA_DROP_BLUE_MIDDLE_RELEASE_PURPLE_X, AutoConstants.CAMERA_DROP_BLUE_MIDDLE_RELEASE_PURPLE_Y, AutoConstants.CAMERA_DROP_BLUE_MIDDLE_RELEASE_PURPLE_HEADING);
                driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                        .splineToSplineHeading(new Pose2d(AutoConstants.CAMERA_DROP_BLUE_MIDDLE_RELEASE_PURPLE_X, AutoConstants.CAMERA_DROP_BLUE_MIDDLE_RELEASE_PURPLE_Y, AutoConstants.CAMERA_DROP_BLUE_MIDDLE_RELEASE_PURPLE_HEADING), AutoConstants.RELEASE_PURPLE_TANGENT)
                        .build();
            } else {
                lastPurpleDropPose = new Pose2d(AutoConstants.CAMERA_DROP_BLUE_RIGHT_RELEASE_PURPLE_X, AutoConstants.CAMERA_DROP_BLUE_LEFT_RIGHT_RELEASE_PURPLE_Y, AutoConstants.CAMERA_DROP_BLUE_RIGHT_RELEASE_PURPLE_HEADING);
                driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                        .splineToSplineHeading(new Pose2d(AutoConstants.CAMERA_DROP_BLUE_RIGHT_RELEASE_PURPLE_X, AutoConstants.CAMERA_DROP_BLUE_LEFT_RIGHT_RELEASE_PURPLE_Y, AutoConstants.CAMERA_DROP_BLUE_RIGHT_RELEASE_PURPLE_HEADING), AutoConstants.RELEASE_PURPLE_TANGENT)
                        .build();
            }

            Action driveToPark = robot.drivebase.drive.actionBuilder(lastPurpleDropPose)
                    .strafeTo(new Vector2d(AutoConstants.CAMERA_DROP_BLUE_PARK_X, AutoConstants.CAMERA_DROP_BLUE_PARK_Y))
                    .build();

            Actions.runBlocking(
                    new SequentialAction(
                            (telemetryPacket) -> { // Extend Arm
                                robot.pixelManipulator.arm.autoExtend();
                                telemetryPacket.addLine("Extending Arm");
                                return false;
                            },
                            new SleepAction(sleepInSeconds),
                            driveToPurpleDrop,
                            (telemetryPacket) -> { // Drop Purple
                                robot.pixelManipulator.claw.releaseServo(robot.pixelManipulator.claw.leftClamp);
                                return false;
                            },
                            new SleepAction(sleepInSeconds),
                            driveToPark

                    )
            );
            break;
        }
    }
}
