package org.firstinspires.ftc.teamcode.subsystems.settings;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadSettings {

    // GAMEPAD 1
    public static final int EXPONENT_MODIFIER = 3;
    public static final double VX_WEIGHT = 1;
    public static final double VY_WEIGHT = 1;
    public static final double VRX_WEIGHT = .92;

    public static final double VX_WEIGHT_SLOW = 0.78;
    public static final double VY_WEIGHT_SLOW = 0.73;
    public static final double VRX_WEIGHT_SLOW = 0.58;

    public static final double GP1_STICK_DEADZONE = 0.05;
    public static final double GP1_TRIGGER_DEADZONE = 0.25;

    // GAMEPAD 2

    public static final double GP2_STICK_DEADZONE = 0.08;
    public static final double GP2_TRIGGER_DEADZONE = 0.25;

    public static final Gamepad.RumbleEffect endgameEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 0.5, 250)
            .addStep(0.5, 0.1, 250)
            .addStep(1.0, 1.0, 500)
            .build();

}
