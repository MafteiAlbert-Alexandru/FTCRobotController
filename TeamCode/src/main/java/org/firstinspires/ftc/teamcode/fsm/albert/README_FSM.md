# Crash course in using the FSM

```java

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.fsm.albert.ButtonTransition;
import org.firstinspires.ftc.teamcode.fsm.albert.SmartFSM;
import org.firstinspires.ftc.teamcode.fsm.albert.SmartState;
import org.firstinspires.ftc.teamcode.fsm.albert.SmartTransition;

import java.util.Random;

class CrashCourse extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Creates a FSM for usage inside of a TeleOP
        SmartFSM fsm = new SmartFSM();


        // Creates a new state
        // IMPORTANT: Note the fsm inside parenthesis and the {}(brackets)
        //      The brackets are a anonymous class implementation which guarantees the uniquness of a state
        //      and a custom implementation
        SmartState state1 = new SmartState(fsm) {
            @Override
            public void update() {
                telemetry.addData("State", 1);
                // move motor or smth
            }
        });

        Smartstate state2 = new SmartState(fsm) {
            @Override
            public void update() {
                telemetry.addData("State", 2);
                // do smth else
            }

        };
        GamepadEx testGamepad = new GamepadEx(gamepad1);

        // Adds a transition which executes on a button
        fsm.addTransition(new ButtonTransition(state1, state2, testGamepad, GamepadKeys.Button.A) {
            @Override
            public void run() {
                telemetry.addLine("Inside transition between 1 and 2");
            }
        });

        // Adds a transition from state 2 to state 1 which executes randomly because fuck you that's why
        fsm.addTransition(new SmartTransition(state2, state1) {
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