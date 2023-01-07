package org.firstinspires.ftc.teamcode.robot.fsm;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Stack;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class FSM {

    HashSet<State> states=new HashSet<State>();
    HashSet<Transition> transitions=new HashSet<Transition>();
    HashMap<State, ArrayList<Transition>> transitionsLookup=new HashMap<State, ArrayList<Transition>>();
    private State currentState = null;


    private ExecutorService executor =null;

    public Lock executionLock = new ReentrantLock();
    Deque<Transition> executionDeque = new LinkedList<Transition>();
    private Telemetry telemetry=null;

    public FSM(Telemetry telemetry)
    {
        this.telemetry=telemetry;
        executor =Executors.newSingleThreadExecutor();
    }

    public FSM()
    {
        executor = Executors.newSingleThreadExecutor();
    }

    /**
     * Adds the state to the FSM
     * @param state The state to be added
     */
    public void add(State state)
    {
        states.add(state);
        state.fsm=this;
    }

    /**
     * Adds a transititon to the FSM
     * @param smartTransition The transition to be added
     */
    public void add(Transition smartTransition)
    {
        transitions.add(smartTransition);
        smartTransition.fsm=this;
    }

    /**
     * Adds the exact same transition between all 'from' to 'to'
     * @param to The final state
     * @param from All possible initial states
     * @param transition The transition to be added
     */
    public void addTransitionsTo(State to, State[] from, Transition transition)
    {
        add(to);
        for (State state: from)
        {
            add(state);
            add(new Transition(state, to) {
                @Override
                public boolean check()
                {
                    return transition.check();
                }
                @Override
                public boolean run() throws InterruptedException{
                    return transition.run();
                }
            });
        }
    }
    public State getCurrentState()
    {
        return  currentState;
    }
    /**
     * This needs to be called for multi-state finite machines before running the fsm
     */
    private boolean buildCalled =false;
    public void build()
    {
        for (State state :
                states) {
            state.init();
        }
        for(Transition transition: transitions)
        {
            transition.init();
        }

        for(Transition transition: transitions)
        {
            ArrayList<Transition> transitionList = transitionsLookup.getOrDefault(transition.from, new ArrayList<>());
            transitionList.add(transition);

            transitionsLookup.put(transition.from, transitionList);
        }
        buildCalled=true;
    }

    public void update(boolean transitionCheck)
    {
        assert buildCalled : "Build not called before update";
        if(currentState!=null)
        {
            currentState.update();
            if(executionDeque.isEmpty())
            {

                if(transitionCheck)
                for(Transition transition: transitionsLookup.getOrDefault(currentState, new ArrayList<>())) {
                    if (transition.check()) {

                        executionDeque.addLast(transition);
                    }
                }
            }else {
                    if(executionLock.tryLock())
                    {
                        executionLock.unlock();
                        executor.execute(()->{
                            executionLock.lock();
                            try {

                                Transition transition = executionDeque.peekFirst();
                                if(transition.run())
                                {
                                    currentState=transition.to;
                                }
                                executionDeque.removeFirst();
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            } finally {
                                executionLock.unlock();
                            }
                        });
                    }



            }


        }

    }
    public void enqueStates(State[] states)
    {
        State state;
        if (executionDeque.peekLast() != null) {
            state = executionDeque.peekLast().to;
        } else {
            state = currentState;
        }

        for(int i=0;i<states.length;i++)
        {
            for (Transition transition:
                 transitionsLookup.getOrDefault(state, new ArrayList<>())) {
                if(transition.to==states[i])
                {
                    executionDeque.add(transition);
                    state=transition.to;
                }
            }
        }
    }
    public void goTo(State target) {
        State state;
        if (executionDeque.peekLast() != null) {
            state = executionDeque.peekLast().to;
        } else {
            state = currentState;
        }
        Stack<Transition> transitions = new Stack<>();
        depthFirstSearch(state,target, new HashMap<>(), transitions);
        executionDeque.addAll(transitions);
    }
    private void depthFirstSearch(State current, State target, HashMap<State, Boolean> visited, Stack<Transition> path)
    {
        visited.put(current, true);
        for(Transition transition: transitionsLookup.getOrDefault(current, new ArrayList<>()))
        {
            if(transition.to==target)
            {
                path.push(transition);
                return;
            }
            if(!visited.getOrDefault(transition.to, false))
            {
                path.push(transition);
                depthFirstSearch(transition.to, target, visited, path);
                if(path.lastElement().to==target) return;
                path.pop();
            }
        }
        visited.put(current,false);
    }
    /**
     * Sets the initial state and adds it to the FSM
     * @param state State to be set as initial and added
     */
    public void setInitialState(State state)
    {
        states.add(state);
        currentState=state;
    }
}
