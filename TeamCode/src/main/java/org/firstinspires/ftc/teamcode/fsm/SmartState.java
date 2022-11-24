package org.firstinspires.ftc.teamcode.fsm;

import java.util.Objects;

public abstract class SmartState {
    void init() {};
     void update() {};


    @Override
    public int hashCode() {
        try {
            return Objects.hash(this.getClass().getMethod("init").hashCode(), this.getClass().getMethod("init").hashCode());
        } catch (NoSuchMethodException e) {
           return 0;
        }

    }
}
