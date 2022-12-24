package org.firstinspires.ftc.teamcode.fsm.albert;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

/**
 * A transition triggered on a button press
 */
public class ReleaseTransition extends Transition {

    private final GamepadEx gamepad;
    private final GamepadKeys.Button button;
    public ReleaseTransition(State from, State to, GamepadEx gamepad, GamepadKeys.Button button)
    {
        super(from, to);
        this.gamepad=gamepad;
        this.button=button;
    }
    public ReleaseTransition(GamepadEx gamepad, GamepadKeys.Button button)
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
        return gamepad.wasJustReleased(button);
    }

    @Override
    public void run() throws InterruptedException {

    }
}
