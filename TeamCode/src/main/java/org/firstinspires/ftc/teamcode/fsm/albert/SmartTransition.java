package org.firstinspires.ftc.teamcode.fsm.albert;

import java.util.Objects;

public abstract class SmartTransition {
    public SmartState from;
    public SmartState to;
    public SmartTransition(SmartState from, SmartState to)
    {
        this.from=from;
        this.to=to;

    }
    public SmartTransition(){}
    void init()
    {

    }
    boolean check()
    {
        return false;
    }
    void run()
    {

    }


    @Override
    public int hashCode() {
        return Objects.hash(from, to);
    }
}
