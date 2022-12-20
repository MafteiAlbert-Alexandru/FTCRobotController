package org.firstinspires.ftc.teamcode.fsm;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.fsm.albert.ButtonTransition;
import org.firstinspires.ftc.teamcode.fsm.albert.FSM;
import org.firstinspires.ftc.teamcode.fsm.albert.MovementTransition;
import org.firstinspires.ftc.teamcode.fsm.albert.State;
import org.firstinspires.ftc.teamcode.fsm.albert.Transition;
import org.firstinspires.ftc.teamcode.subsystem.ClampSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.MovementSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SmartSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SubsystemData;
import org.firstinspires.ftc.teamcode.subsystem.TransferSubsystem;

public class Robot {

    //region Subsystem

    public  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public  TransferSubsystem transferSubsystem = new TransferSubsystem();
    public  MovementSubsystem movementSubsystem = new MovementSubsystem();
    public  SliderSubsystem sliderSubsystem = new SliderSubsystem();
    public ClampSubsystem clampSubsystem = new ClampSubsystem();
    public   SensorSubsystem sensorSubsystem = new SensorSubsystem();

    //endregion

    //region State
    public State initialState;
    public State waitingState;
    public State loadedState;
    public State upperState;
    public State mediumState;
    public State lowerState;
    public State groundState;
    public State frontWaitingState;
    public State movingState;
    public State homingState;
    //endregion

    //region FSM
    public final FSM SliderAndClampingFSM = new FSM();
    public final FSM MovementFSM = new FSM();
    //endregion

    private final boolean autonomous;
    private final GamepadEx driverGamepad;
    private final GamepadEx operatorGamepad;

    public Robot(OpMode opMode, boolean autonomous) throws IllegalAccessException {
        this.autonomous = autonomous;

        SmartSubsystem.initAllSubsystems(this, opMode);
        //initializeaza toate subsystemele

        //defintesc state urile
        initialState = new State(SliderAndClampingFSM, "initialState") {
        };
        waitingState = new State(SliderAndClampingFSM, "waitingState") {
        };
        loadedState = new State(SliderAndClampingFSM, "loadedState") {
        };
        upperState = new State(SliderAndClampingFSM, "upperState") {
        };
        mediumState = new State(SliderAndClampingFSM, "mediumState") {
        };
        lowerState = new State(SliderAndClampingFSM, "lowerState") {
        };
        groundState = new State(SliderAndClampingFSM, "groundState") {
        };
        frontWaitingState = new State(SliderAndClampingFSM, "frontWaitingState") {
        };

        //initializez gamepad-urile
        driverGamepad = new GamepadEx(opMode.gamepad1);
        operatorGamepad = new GamepadEx(opMode.gamepad2);

        //imi setez un state initial
        SliderAndClampingFSM.setInitialState(initialState);


        SliderAndClampingFSM.add(new ButtonTransition(initialState, waitingState, operatorGamepad, GamepadKeys.Button.Y) {
            //adaug un state intre cel initial si cel de waiting
            //isi ia trigger de la butonul Y (ala de sus de pe partea dreapta pt cei care nu le stiu dupa litere/daca esti hater la alea logitech)

            @Override
            public void run() throws InterruptedException {
                sliderSubsystem.goTo(SliderSubsystem.SafePos);
                clampSubsystem.goTo(ClampSubsystem.BackwardPos);
                clampSubsystem.release();
                //hook-ul se duce in SafePos si pe spate (adica in robot)
            }
        });

        SliderAndClampingFSM.add(new Transition(waitingState, loadedState) {
            //Tranzitie care isi poate lua trigger ori de la buton ori de la alti factori (ex senzori)

            @Override
            //Asta verifica daca isi ia trigger tranzitia
            public boolean check() {
                // IF SENSOR TODO
                return operatorGamepad.wasJustPressed(GamepadKeys.Button.Y);
            }

            @Override
            public void run() throws InterruptedException {
                //daca e sus mergi jos (vezi in codul sursa)
                if(transferSubsystem.isUp()) transferSubsystem.goDown();
                clampSubsystem.release(); //dau drumul la hook (siguranta)
                sliderSubsystem.goTo(SliderSubsystem.LoadPos); //duc slider-ul in con (o bag tare)
                clampSubsystem.clamp();//imi deschid carligul/prind conul
            }
        });

        //restul se inteleg daca ai citit si codul sursa de la subsysteme
        SliderAndClampingFSM.add(new ButtonTransition(waitingState, frontWaitingState, operatorGamepad, GamepadKeys.Button.LEFT_STICK_BUTTON) {
            @Override
            public void run() throws InterruptedException {
                clampSubsystem.goTo(ClampSubsystem.ForwardPos);
            }
        });

        State[] sliderSafeStates = {loadedState, upperState, mediumState, lowerState, groundState};
        SliderAndClampingFSM.addTransitionsTo(upperState, sliderSafeStates, new ButtonTransition(operatorGamepad, GamepadKeys.Button.DPAD_UP) {
            @Override
            public void run() throws InterruptedException {
                sliderSubsystem.goTo(SliderSubsystem.HighPos);
                clampSubsystem.goTo(ClampSubsystem.ForwardPos);
            }
        });
        SliderAndClampingFSM.addTransitionsTo(mediumState, sliderSafeStates, new ButtonTransition(operatorGamepad, GamepadKeys.Button.DPAD_RIGHT) {

            @Override
            public void run() throws InterruptedException {
                sliderSubsystem.goTo(SliderSubsystem.MediumPos);
                clampSubsystem.goTo(ClampSubsystem.ForwardPos);
            }
        });
        SliderAndClampingFSM.addTransitionsTo(lowerState, sliderSafeStates, new ButtonTransition(operatorGamepad, GamepadKeys.Button.DPAD_LEFT) {
            @Override
            public void run() throws InterruptedException {
                if (sliderSubsystem.isSafe()) clampSubsystem.goTo(ClampSubsystem.ForwardPos);
                else {
                    sliderSubsystem.goTo(SliderSubsystem.SafePos);
                    clampSubsystem.goTo(ClampSubsystem.ForwardPos);
                }
                sliderSubsystem.goTo(SliderSubsystem.LowPos);

            }
        });
        SliderAndClampingFSM.addTransitionsTo(groundState, sliderSafeStates, new ButtonTransition(operatorGamepad, GamepadKeys.Button.DPAD_DOWN) {
            @Override
            public void run() throws InterruptedException {
                if (sliderSubsystem.isSafe()) clampSubsystem.goTo(ClampSubsystem.ForwardPos);
                else {
                    sliderSubsystem.goTo(SliderSubsystem.SafePos);
                    clampSubsystem.goTo(ClampSubsystem.ForwardPos);
                }

            }
        });
        State[] outsideStates = new State[]{groundState, frontWaitingState, lowerState, mediumState, upperState};
        SliderAndClampingFSM.addTransitionsTo(waitingState, outsideStates, new Transition() {
            Long lastTimeSinceClamped = null;

            @Override
            public boolean check() {

                if (lastTimeSinceClamped != null) {
                    if (System.currentTimeMillis() - lastTimeSinceClamped > 500) {
                        lastTimeSinceClamped = null;
                        return true;
                    }
                } else {
                    if (!clampSubsystem.isClamping()) {
                        lastTimeSinceClamped = System.currentTimeMillis();
                    }
                    return false;
                }
                return false;


            }

            @Override
            public void run() throws InterruptedException {
                if (sliderSubsystem.isSafe()) {
                    clampSubsystem.goTo(ClampSubsystem.BackwardPos);
                    sliderSubsystem.goTo(SliderSubsystem.SafePos);
                } else {
                    sliderSubsystem.goTo(SliderSubsystem.SafePos);
                    clampSubsystem.goTo(ClampSubsystem.BackwardPos);
                }
            }
        });
        SliderAndClampingFSM.build();
//        WebcamUtil webcamUtil = new WebcamUtil(opMode.hardwareMap);
//
//        JunctionAdjuster junctionAdjuster = new JunctionAdjuster(webcamUtil, 2.54, opMode.telemetry);
//        webcamUtil.registerListener(junctionAdjuster);
//        webcamUtil.start(true);
        opMode.telemetry.update();
        Robot robot = this;
        movingState = new State(MovementFSM, "movementState") {
            public void update() {
                movementSubsystem.run(new SubsystemData() {{
                    this.driverGamepad = robot.driverGamepad;
                }});
            }
        };
        homingState = new State(MovementFSM, "homingState") {
            public void update() {
//                Vector2d direction = junctionAdjuster.value(0.7, new JunctionAdjuster.Vec2(-10.2, 4.4));
//                movementSubsystem.move(direction.getY(), direction.getX(), 0);
            }
        };
        MovementFSM.add(new ButtonTransition(movingState, homingState, driverGamepad, GamepadKeys.Button.A) {
        });
        MovementFSM.add(new MovementTransition(homingState, movingState, driverGamepad) {
        });
        MovementFSM.setInitialState(movingState);
        MovementFSM.build();
        subsystemData.driverGamepad=driverGamepad;
        subsystemData.operatorGamepad=operatorGamepad;
    }
    private SubsystemData subsystemData = new SubsystemData();
    public void update() throws InterruptedException {
        if(!this.autonomous)
        {
            subsystemData.driverGamepad.readButtons();
            subsystemData.operatorGamepad.readButtons();
        }

        SliderAndClampingFSM.update(!this.autonomous);

        sliderSubsystem.run(subsystemData);
        clampSubsystem.run(subsystemData);

        if(!this.autonomous)
        {
            MovementFSM.update(true);

            sensorSubsystem.run(subsystemData);
            if(sensorSubsystem.toFlip())
            {
                transferSubsystem.lift();
            }
            transferSubsystem.run(subsystemData);
            intakeSubsystem.run(subsystemData);
        }

    }
}