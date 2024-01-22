package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoUtil {

    // Check if a servo is close to a specific position with a given proximity
    public static boolean isClose(Servo servo, double pos, double proximity) {
        return servo != null && Math.abs(servo.getPosition() - pos) < proximity;
    }

    // Incrementally adjust positions of two servos
    public static void increment(Servo leftServo, Servo rightServo, double input) {
        if (leftServo != null && rightServo != null) {
            leftServo.setPosition(leftServo.getPosition() + input);
            rightServo.setPosition(rightServo.getPosition() + input);
        }
    }

    // Incrementally adjust positions of one servos
    public static void increment(Servo servo, double input) {
        if (servo != null) {
            servo.setPosition(servo.getPosition() + input);
        }
    }
}
