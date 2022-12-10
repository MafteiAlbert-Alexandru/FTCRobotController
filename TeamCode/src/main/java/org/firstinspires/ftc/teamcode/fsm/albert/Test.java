package org.firstinspires.ftc.teamcode.fsm.albert;

public class Test {
    public  static void main(String[] args)
    {
        FSM fsm = new FSM();


        State state1 = new State(fsm, "1") {
            @Override
            public void update() {
                System.out.println("STATE1");
            }
        };

        State state2 = new State(fsm, "2") {
            @Override
            public void update() {
                System.out.println("STATE2");
            }
        };
        State state3 = new State(fsm, "3") {
            @Override
            public void update() {
                System.out.println("STATE3");
            }
        };
        State state4 = new State(fsm, "4") {
            @Override
            public void update() {
                System.out.println("STATE4");
            }
        };

        State state5 = new State(fsm, "5") {
            @Override
            public void update() {
                System.out.println("STATE5");
            }
        };



        fsm.setInitialState(state3);
        fsm.add(new Transition(state1, state2) {});
        fsm.add(new Transition(state1, state3) {});
        fsm.add(new Transition(state2, state4) {});
        fsm.add(new Transition(state3, state4) {});
        fsm.add(new Transition(state2, state5) {});
        fsm.add(new Transition(state4, state1) {});

        fsm.build();
        fsm.goTo(state5);


    }
}
