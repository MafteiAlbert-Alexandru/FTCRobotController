package org.firstinspires.ftc.teamcode.fsm.albert;

/**
 * Represents a state that is null, has no value or smth
 */
public abstract class NullState extends SmartState {

    public NullState()
    {
        super();
    }
    public NullState(SmartFSM fsm) {
        super(fsm);
    }
}
