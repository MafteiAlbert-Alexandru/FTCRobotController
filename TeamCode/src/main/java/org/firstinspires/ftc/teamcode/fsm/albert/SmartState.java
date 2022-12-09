package org.firstinspires.ftc.teamcode.fsm.albert;

import java.util.Objects;

/**\
 * Class that represents a state
 */
public abstract class SmartState {
    public void init() {};
    public void update() {};

    /**
     * Create a SmartState
     */
    public SmartState() {}

    /**
     * Create a SmartState and adds it to the fsm automatically
     * @param fsm The SmartFSM the state needs to be added to
     */
    public SmartState(SmartFSM fsm) {fsm.addState(this);}

    /**
     * Way of differentiating between states(it's based on what they do
     * TODO: Add other differentiator
     */
    @Override
    public int hashCode() {
        try {
            return Objects.hash(this.getClass().getMethod("init").hashCode(), this.getClass().getMethod("update").hashCode());
        } catch (NoSuchMethodException e) {
           return 0;
        }

    }

}
