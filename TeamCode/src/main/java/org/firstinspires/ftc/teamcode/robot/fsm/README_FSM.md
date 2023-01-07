# Crash course in using the FSM

```java

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.fsm.ButtonTransition;
import org.firstinspires.ftc.teamcode.robot.fsm.FSM;
import org.firstinspires.ftc.teamcode.robot.fsm.State;
import org.firstinspires.ftc.teamcode.robot.fsm.Transition;

import java.util.Random;

class CrashCourse extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Creates a FSM for usage inside of a TeleOP
        FSM fsm = new FSM();


        // Creates a new state
        // IMPORTANT: Note the fsm inside parenthesis and the {}(brackets)
        //      The brackets are a anonymous class implementation which guarantees the uniquness of a state
        //      and a custom implementation
        State state1 = new State(fsm) {
            @Override
            public void update() {
                telemetry.addData("State", 1);
                // move motor or smth
            }
        });

        Smartstate state2 = new State(fsm) {
            @Override
            public void update() {
                telemetry.addData("State", 2);
                // do smth else
            }

        };
        GamepadEx testGamepad = new GamepadEx(gamepad1);

        // Adds a transition which executes on a button
        fsm.add(new ButtonTransition(state1, state2, testGamepad, GamepadKeys.Button.A) {
            @Override
            public void run() {
                telemetry.addLine("Inside transition between 1 and 2");
            }
        });

        // Adds a transition from state 2 to state 1 which executes randomly because fuck you that's why
        fsm.add(new Transition(state2, state1) {
            Random random = new Random();

            @Override
            public boolean check() {
                return random.nextBoolean();
            }
        });

        // Build the fsm
        fsm.build();

        while (opModeIsActive() && !isStopRequested()) {
            testGamepad.readButtons();
            fsm.update();
            telemetry.update();
        }

    }
};


```