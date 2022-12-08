package org.firstinspires.ftc.teamcode.fsm.albert;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class ButtonTransition extends SmartTransition{

    private GamepadEx gamepad;
    private GamepadKeys.Button button;
    public ButtonTransition(SmartState from, SmartState to, GamepadEx gamepad, GamepadKeys.Button button)
    {
        super(from, to);
        this.gamepad=gamepad;
        this.button=button;
    }
    public ButtonTransition(GamepadEx gamepad, GamepadKeys.Button button)
    {
        super();
        this.gamepad=gamepad;
        this.button=button;
    }
    @Override
    public void init() {

    }

    @Override
    public boolean check() {
        return gamepad.wasJustPressed(button);
    }

    @Override
    public void run() {

    }
}
