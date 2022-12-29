package org.firstinspires.ftc.teamcode.fsm.albert;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

/**
 * A transition triggered on a button press
 */
public class ButtonTransition extends Transition {

    private final GamepadEx gamepad;
    private  GamepadKeys.Button button=null;
    private GamepadKeys.Trigger trigger=null;
    public ButtonTransition(State from, State to, GamepadEx gamepad, GamepadKeys.Button button)
    {
        super(from, to);
        this.gamepad=gamepad;
        this.button=button;
    }
    public ButtonTransition(State from, State to, GamepadEx gamepad, GamepadKeys.Trigger trigger)
    {
        super(from, to);
        this.gamepad=gamepad;
        this.trigger=trigger;
    }
    public ButtonTransition(GamepadEx gamepad, GamepadKeys.Button button)
    {
        super();
        this.gamepad=gamepad;
        this.button=button;
    }
    public ButtonTransition(GamepadEx gamepad, GamepadKeys.Trigger trigger)
    {
        super();
        this.gamepad=gamepad;
        this.trigger=trigger;
    }
    @Override
    public void init() {

    }

    @Override
    public boolean check() {
        if(button!=null) return gamepad.wasJustPressed(button);
        else if(trigger!=null) return gamepad.getTrigger(trigger)>0.1;
        return false;
    }

    @Override
    public boolean run() throws InterruptedException {
        return true;
    }
}
