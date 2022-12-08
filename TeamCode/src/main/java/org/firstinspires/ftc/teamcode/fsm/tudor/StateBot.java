package org.firstinspires.ftc.teamcode.fsm.tudor;


import org.firstinspires.ftc.teamcode.subsystem.ClampSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;

//Tudor e frustrat ca SM-ul lui Albert nu merge bine
//Ascult Oscar pe fundal cand fac asta deci codul cred ca o iesit bine
public class StateBot{

    SliderSubsystem sliderSubsystem;
    ClampSubsystem clamp;

    public StateBot(SliderSubsystem sliderSubsystem, ClampSubsystem clamp){
        this.sliderSubsystem = sliderSubsystem;
        this.clamp = clamp;
    }

    //Astea is "state"-urile pe care robot-ul le poate avea
    //Cand ai nevoie de inca un state doar il adaugi aici
    public enum BotState{
        Idle, //Robotul nu e controlat de SM (termenul e folosit in jocuri cand player-ul nu face nimic)
        Start, //Asa incepe robotul
        Ground, //Cand glisiera e la nivelul Ground Junction-ului
        Low, //Cand glisiera e la nivelul Low Junction-ului
        Mid, //Cand glisiera e la nivelul Mid Junction-ului
        High, //Cand glisiera e la nivelul High Junction-ului
        Empty, //Cand n-ai con (cred ca nu e nevoie de asta)
        Grip, //Cand esti bazat si ai prins conu
        Aim, //Cand vrei sa "tintesti" (aka sa te pozitionezi) ca sa iei contul cu carligul magic a lui Albert
        Load, //Cand vre sa prinzi con
        Release //Cand vrei sa lasi con
    }

    //Stim ce "state"-uri poate avea robotul doar ca trebuie sa stim si ce state are
    //(asta facem prin a avea o variabila ce tine minte in ce state suntem)
    public BotState botState = BotState.Mid;
    BotState previousState = botState;

    //region beta-test

//    public enum SliderSubsystemState{
//        Start,
//        Ground,
//        Low,
//        Mid,
//        High,
//        Manual
//    }
//
//    SliderSubsystemState sliderState = SliderSubsystemState.Start;
//
//    public enum HookState{
//        Open,
//        Close
//    }
//
//    HookState hookState = HookState.Close;
//
//    public enum ArmState {
//        Retract,
//        Deploy
//    }
//
//    ArmState armState = ArmState.Retract;

    //endregion

    public void update(){
        if(botState != previousState){
            previousState = botState;
            TransitionState(botState);
        }
    }

    void TransitionState(BotState newState){
        switch (newState){
            case Ground:
                groundCase();
                break;
            case Low:
                lowCase();
                break;
            case Mid:
                midCase();
                break;
            case High:
                highCase();
                break;
            case Load:
                loadCase();
                break;
            case Release:
                releaseCase();
                break;
//            case Empty:
//                sliderSubsystem.setTarget(SliderSubsystem.groundPos);
//                break;
//            case Grip:
//                sliderSubsystem.setTarget(SliderSubsystem.groundPos);
//                break;
            default: break;
        }


    }
//
//    void ClampUpdate(){
//        switch (hookState){
//            case Open: clamp.release();
//                break;
//            case Close: clamp.clamp();
//                break;
//        }
//
//        switch (armState){
//            case Retract: clamp.goToBackward();
//                break;
//            case Deploy: clamp.goToForward();
//                break;
//        }
//    }

    void groundCase() {
        sliderSubsystem.setTarget(SliderSubsystem.groundPos);
    }
    void lowCase(){
        sliderSubsystem.setTarget(SliderSubsystem.lowPos);
    }
    void midCase(){
        sliderSubsystem.setTarget(SliderSubsystem.midPos);
    }
    void highCase(){
        sliderSubsystem.setTarget(SliderSubsystem.highPos);
    }

    void loadCase(){
        sliderSubsystem.setTarget(SliderSubsystem.loadPos);
        sleepThread(100);
        clamp.clamp();
    }
    void releaseCase(){
        clamp.release();
        sleepThread(100);
        if(!isSafe()){
            SafeSlider();
            while(!isSafe()){}
            clamp.goToBackward();
        }
        else{
            clamp.goToBackward();
            SafeSlider();
        }
    }

    public void sleepThread(int mil){

        try {
            Thread.sleep(mil);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

    public void SafeSlider(){
        sliderSubsystem.target = SliderSubsystem.safePos;
    }

    public boolean isSafe() {return sliderSubsystem.getPosition() >= SliderSubsystem.safePos;}

    public void setState(BotState state) {
        botState = state;
    }

}



//Foarte smash codu asta
//L-am scris intr o seara nu ma judecati