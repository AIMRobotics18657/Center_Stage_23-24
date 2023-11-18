package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Mechanism;

public class PAL extends Mechanism {
    Servo PAL;

    public String PaperAirplane = "PAL";

    @Override
    public void init(HardwareMap hwMap) {
        PAL = hwMap.get(Servo.class, PaperAirplane);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            PAL.setPosition(1);
        } else if (gamepad.dpad_down) {
            PAL.setPosition(-1);
        } else {
            PAL.setPosition(0);
        }
    }
}


