package org.firstinspires.ftc.teamcode.fsm;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class ButtonTransition extends SmartTransition{

    private GamepadEx gamepad;
    private GamepadKeys button;
    public ButtonTransition(GamepadEx gamepad, GamepadKeys.Button button)
    {
        if(gamepad.wasJustPressed(button))
        {

        }
    }
    @Override
    public void init() {

    }

    @Override
    public boolean check() {
        return false;
    }

    @Override
    public void run() {

    }
}
