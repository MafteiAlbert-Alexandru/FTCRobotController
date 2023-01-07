package org.firstinspires.ftc.teamcode.robot.fsm;

import java.util.Objects;

/**
 * Represents a transition between two states
 */
public abstract class Transition{
    public State from;
    public State to;
    public FSM fsm;
    public boolean humanInput = false;

    /**
     * Creates a SmartTransition between two SmartStates
     * @param from From state
     * @param to To state
     */
    public Transition(State from, State to)
    {
        this.from=from;
        this.to=to;

    }
    public Transition(){}


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
     * Function called by the fsm(in parallel) on execution of this transition
     * Override for custom implentation
     */
    public boolean run() throws InterruptedException
    {
        return true;
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
