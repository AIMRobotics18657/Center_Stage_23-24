package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Mechanism;

public class MatchTimer extends Mechanism {

    ElapsedTime timer = new ElapsedTime();
    @Override
    public void init(HardwareMap hwMap) {
        timer.reset();
    }

    @Override
    public void loop(Gamepad gamepad) {

    }

    public void start() {
        timer.reset();
    }

    public double getTime() {
        return timer.seconds();
    }
}
