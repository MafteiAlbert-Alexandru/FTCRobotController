package org.firstinspires.ftc.teamcode.fsm.albert;

import java.util.Objects;

/**
 * Represents a transition between two states
 */
public abstract class SmartTransition {
    public SmartState from;
    public SmartState to;

    /**
     * Creates a SmartTransition between two SmartStates
     * @param from From state
     * @param to To state
     */
    public SmartTransition(SmartState from, SmartState to)
    {
        this.from=from;
        this.to=to;

    }
    public SmartTransition(){}


    /**
     * Is run on the init of the SmartTransition
     * Implement this to provide values needed per specific transitions
     */
    public void init()
    {

    }

    /**
     * Checks if this transition can be run through, acts as a condition between two states
     * Implement this to provide transition conditions
     * @return True if transition
     */
    public boolean check()
    {
        return false;
    }

    /**
     * Code to be executed on the transition
     * Implement this to add custom behavior
     */
    public void run()
    {

    }


    /**
     * Comparison is done based on the nodes
     * No multiple transitions with the same direction between two nodes can exist
     */
    @Override
    public int hashCode() {
        return Objects.hash(from, to);
    }
}
