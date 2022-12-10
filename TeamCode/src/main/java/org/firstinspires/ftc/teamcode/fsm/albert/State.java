package org.firstinspires.ftc.teamcode.fsm.albert;

import java.util.Objects;

/**\
 * Class that represents a state
 */
public abstract class State {

    private String name=null;
    public FSM fsm;
    public void init() {};
    public void update() {};


    public State()
    {
        this.name=getClass().getName();
    }

    public State(String name) {
        this.name=name;
    }

    public State(FSM fsm, String name) {
        this.fsm=fsm;
        fsm.add(this);
        name=name;
    }

    public State(FSM fsm)
    {
        this.fsm = fsm;
        fsm.add(this);
        this.name=getClass().getName();
    }


    @Override
    public int hashCode() {
        try {
            return Objects.hash(this.getClass().getMethod("init").hashCode(), this.getClass().getMethod("update").hashCode());
        } catch (NoSuchMethodException e) {
           return 0;
        }

    }

}
