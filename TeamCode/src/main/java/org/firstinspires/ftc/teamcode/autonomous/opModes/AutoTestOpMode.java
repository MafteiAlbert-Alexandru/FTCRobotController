package org.firstinspires.ftc.teamcode.autonomous.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AutonomusOpMode;
import org.firstinspires.ftc.teamcode.subsystem.ClampSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;

@Autonomous
public class AutoTestOpMode extends AutonomusOpMode {
    private SliderSubsystem sliderSubsystem;
    private ClampSubsystem clampSubsystem;


    @Override
    public void Initialization() {
        sliderSubsystem = new SliderSubsystem(this);
        clampSubsystem = new ClampSubsystem(this);
    }

    @Override
    public void ActiveOpModeLoop() throws InterruptedException{
        sliderSubsystem.update();
        clampSubsystem.update();
    }
}
