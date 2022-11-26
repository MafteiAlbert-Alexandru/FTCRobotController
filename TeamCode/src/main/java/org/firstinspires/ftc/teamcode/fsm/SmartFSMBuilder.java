package org.firstinspires.ftc.teamcode.fsm;

import android.transition.Transition;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;

public class SmartFSMBuilder {
    private final SmartFSM fsm;
    private ArrayList<SmartState> states=null;
    private ArrayList<SmartTransition> transitions=null;

    public SmartFSMBuilder(SmartFSM fsm)
    {
        this.fsm=fsm;
        this.states=new ArrayList<>();
        this.transitions=new ArrayList<>();
    }
    private boolean setFirstStateInitial=false;
    public SmartFSMBuilder initial(SmartState state)
    {
        if(toNextState)
        {
            toNextState=false;
            transitions.get(transitions.size()-1).to=state;
        }
        setFirstStateInitial=true;
        states.add(state);
        return this;
    }
    public SmartFSMBuilder add(SmartState state)
    {
        if(toNextState)
        {
            toNextState=false;
            transitions.get(transitions.size()-1).to=state;
        }

        states.add(state);
        return this;
    }

    private boolean toNextState=false;
    public SmartFSMBuilder toNext(SmartTransition transition)
    {
        toNextState=true;
        transition.from=states.get(states.size()-1);
        transitions.add(transition);
        return this;
    }
    public SmartFSMBuilder between(SmartTransition fromTransition, SmartTransition toTransition)
    {
        //TODO: Implement between transitions
        return this;
    }


    public void build()
    {

        for(SmartState state: states)
            fsm.addState(state);
        for(SmartTransition transition:transitions)
            fsm.addTransition(transition);
        if(setFirstStateInitial)
            fsm.setInitialState(states.get(0));
    }


    // TODO: Implement sub-builders for better and easier instantiation
}