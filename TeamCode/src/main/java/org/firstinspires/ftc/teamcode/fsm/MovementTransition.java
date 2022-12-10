package org.firstinspires.ftc.teamcode.fsm;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.fsm.albert.State;
import org.firstinspires.ftc.teamcode.fsm.albert.Transition;

public class MovementTransition extends Transition {
    private GamepadEx gamepad;
    public MovementTransition(State from, State to, GamepadEx gamepad)
    {
        super(from, to);
        this.gamepad=gamepad;
    }
    public MovementTransition(GamepadEx gamepad)
    {
        super();
        this.gamepad=gamepad;
    }
    @Override
    public void init() {

    }

    @Override
    public boolean check() {
        return Math.abs(gamepad.getLeftX())>=0.1 || Math.abs(gamepad.getLeftY())>=0.1 || Math.abs(gamepad.getRightX())>=0.1 || Math.abs(gamepad.getRightY())>=0.1;
    }

    @Override
    public void run() {

    }
}
