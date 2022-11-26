package org.firstinspires.ftc.teamcode.fsm;

import java.util.Objects;

public abstract class SmartTransition {
    public SmartState from;
    public SmartState to;
    public SmartTransition(SmartState from, SmartState to)
    {
        this.from=from;
        this.to=to;

    }
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
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        SmartTransition that = (SmartTransition) o;
        return Objects.equals(from, that.from) && Objects.equals(to, that.to);
    }

    @Override
    public int hashCode() {
        return Objects.hash(from, to);
    }
}
