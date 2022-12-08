package org.firstinspires.ftc.teamcode.fsm.albert;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;

public class SmartFSM implements Callable<Void> {
    HashSet<SmartState> states=new HashSet<SmartState>();
    HashSet<SmartTransition> transitions=new HashSet<SmartTransition>();
    HashMap<SmartState, ArrayList<SmartTransition>> transitionsLookup=new HashMap<SmartState, ArrayList<SmartTransition>>();
    private SmartState currentState = null;
    private ExecutorService threadPool=null;

    public void addState(SmartState state)
    {
        states.add(state);
    }

    public void addTransition(SmartTransition smartTransition)
    {
        transitions.add(smartTransition);
    }
    public void addTransitionsTo(SmartState to, SmartState[] from, SmartTransition transition)
    {
        states.add(to);
        for (SmartState state: from)
        {
            states.add(state);
            transitions.add(new SmartTransition(state, to) {
                @Override
                public boolean check()
                {
                    return transition.check();
                }
                @Override
                void run() {
                    transition.run();
                }
            });
        }
    }
    /**
     * This needs to be called for multi-state finite machines before running the fsm
     */
    public void build()
    {
        for (SmartState state :
                states) {
            state.init();
        }
        for(SmartTransition transition: transitions)
        {
            transition.init();
        }

        for(SmartTransition transition: transitions)
        {
            ArrayList<SmartTransition> transitionList = transitionsLookup.getOrDefault(transition.from, new ArrayList<>());
            transitionList.add(transition);

            transitionsLookup.put(transition.from, transitionList);
        }
    }

    public void update()
    {
        if(currentState!=null)
            currentState.update();

        for(SmartTransition transition: transitionsLookup.getOrDefault(currentState, new ArrayList<>()))
        {
            if(transition.check())
            {
                transition.run();
                currentState=transition.to;
            }
        }
    }
    public void setInitialState(SmartState state)
    {
        states.add(state);
        currentState=state;
    }

    public SmartFSMBuilder builder()
    {
        return new SmartFSMBuilder(this);
    }

    @Override
    public Void call() throws Exception {
        update();
        return null;
    }
}
