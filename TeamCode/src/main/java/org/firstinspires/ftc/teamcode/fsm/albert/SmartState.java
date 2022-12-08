package org.firstinspires.ftc.teamcode.fsm.albert;

import java.util.Objects;

public abstract class SmartState {
    void init() {};
    void update() {};
    public SmartState() {}
    public SmartState(SmartFSM fsm) {fsm.addState(this);}
    @Override
    public int hashCode() {
        try {
            return Objects.hash(this.getClass().getMethod("init").hashCode(), this.getClass().getMethod("update").hashCode());
        } catch (NoSuchMethodException e) {
           return 0;
        }

    }
}
