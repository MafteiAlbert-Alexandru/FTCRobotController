package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.ClampSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderV2Subsystem;

public class AutoFSM{
    int nrCone = 6;

    SliderSubsystem sliderSubsystem;
    ClampSubsystem clampSubsystem;
    LinearOpMode opMode;
    boolean isReady = false;

    public enum AutoState{
        Start,
        DropCone,
        StackPickUp,
        Park
    }

    public AutoState autoState = AutoState.Start;

    Thread AutoFsmThread = new Thread(() -> update(opMode));

    public AutoFSM(LinearOpMode opMode, SliderSubsystem sliderSubsystem, ClampSubsystem clampSubsystem){
        this.opMode = opMode;
        this.clampSubsystem = clampSubsystem;
        this.sliderSubsystem = sliderSubsystem;
        AutoFsmThread.start();
    }

    public void setState(AutoState state){
        isReady = false;
        autoState = state;
    }

    public void update(LinearOpMode opMode){
        try {
            clampSubsystem.update();
            opMode.waitForStart();
            while (opMode.opModeIsActive()){
                sliderSubsystem.update();
                clampSubsystem.update();
                runState();
                opMode.telemetry.addData("state", autoState.toString());
                opMode.telemetry.update();
            }
        } catch (InterruptedException e) {
            opMode.telemetry.addLine(e.toString());
        }
    }

    public void runState() throws InterruptedException {
        if (!isReady) {
            isReady = true;
            if(autoState == AutoState.Start) PreLoad();
            else if(autoState == AutoState.DropCone) JunctionDrop();
            else if(autoState == AutoState.StackPickUp) StackPickUp();
//            else if(autoState == AutoState.Park) Park();
        }
    }


    public void PreLoad() {
        try {
            sliderSubsystem.goTo(SliderV2Subsystem.LowPos);
            clampSubsystem.goTo(ClampSubsystem.BackwardPos);
            sliderSubsystem.goTo(SliderV2Subsystem.LoadPos);
            clampSubsystem.clamp();
            sliderSubsystem.goTo(SliderV2Subsystem.HighPos);
        }
        catch (InterruptedException e) {
            opMode.telemetry.addLine(e.toString());
        }
    }

    public void StackPickUp(){

        switch (nrCone) {
            case 5:
                sliderSubsystem.goTo(SliderV2Subsystem.cone5Pos);
                break;
            case 4:
                sliderSubsystem.goTo(SliderV2Subsystem.cone4Pos);
                break;
            case 3:
                sliderSubsystem.goTo(SliderV2Subsystem.cone3Pos);
                break;
            case 2:
                sliderSubsystem.goTo(SliderV2Subsystem.cone2Pos);
                break;
            case 1:
                sliderSubsystem.goTo(SliderV2Subsystem.cone1Pos);
                break;
        }
    }
    public void JunctionDrop() throws InterruptedException {
        nrCone--;
        clampSubsystem.release();
        sleep(200);
        clampSubsystem.goTo(ClampSubsystem.BackwardPos);
        sliderSubsystem.goTo(SliderV2Subsystem.LowPos);
        clampSubsystem.goTo(ClampSubsystem.ForwardPos);
        sliderSubsystem.goTo(SliderV2Subsystem.AimPos);
    }

}
