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


    /**
     * Adds the state to the FSM
     * @param state The state to be added
     */
    public void addState(SmartState state)
    {
        states.add(state);
    }

    /**
     * Adds a transititon to the FSM
     * @param smartTransition The transition to be added
     */
    public void addTransition(SmartTransition smartTransition)
    {
        transitions.add(smartTransition);
    }

    /**
     * Adds the exact same transition between all 'from' to 'to'
     * @param to The final state
     * @param from All possible initial states
     * @param transition The transition to be added
     */
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
                public void run() {
                    transition.run();
                }
            });
        }
    }
    /**
     * This needs to be called for multi-state finite machines before running the fsm
     */
    private boolean buildCalled =false;
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
        buildCalled=true;
    }

    public void update()
    {
        assert buildCalled : "Build not called before update";
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

    /**
     * Sets the initial state and adds it to the FSM
     * @param state State to be set as initial and added
     */
    public void setInitialState(SmartState state)
    {
        states.add(state);
        currentState=state;
    }

    /**
     * Returns a builder used to shorten creation
     * @return Builder
     */
    public SmartFSMBuilder builder()
    {
        return new SmartFSMBuilder(this);
    }


    /**
     * Used for possible parallelism
     * @return nothing
     * @throws Exception
     */
    @Override
    public Void call() throws Exception {
        update();
        return null;
    }
}
