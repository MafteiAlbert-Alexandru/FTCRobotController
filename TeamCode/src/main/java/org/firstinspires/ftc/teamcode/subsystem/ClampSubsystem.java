package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.customHardware.ToggleButton;

@Config
public class ClampSubsystem extends SmartSubsystem{
    private Servo baseServo;
    private ToggleButton clampButton;
    private Servo armPullServo;
    public static double position=0;
    public static double clampOffset=0.2;
    public static double unclampedOffset=0.2;
    public static double initialPosition=0.4;
    public static double ForwardPos = 0.1;
    public static double BackwardPos = 0.62;
    private boolean clamping = false;
    public boolean isClamping()
    {return  clamping;
    }
    public void clamp()
    {
        clamping=true;
        setPosition(position);
    }

    public void release()
    {
        clamping=false;
        setPosition(position);
    }
    public void goTo(double position) throws InterruptedException {
        double initialPosition = baseServo.getPosition();
        double delta = Math.abs(position-initialPosition);
        double time = delta * 300 / 240.0 *1000;
        setPosition(position);
        Thread.sleep((long) time);
    }
    public void setPosition(double position)
    {
        this.position=position;
        baseServo.setPosition(position);
        if(clamping) armPullServo.setPosition(baseServo.getPosition()+clampOffset-unclampedOffset);
        else armPullServo.setPosition(baseServo.getPosition()+clampOffset);
    }
    public void goToForward() throws InterruptedException {
        baseServo.setPosition(ForwardPos);


    }
    public void goToBackward() throws InterruptedException {
        baseServo.setPosition(0.6);
        if(clamping) armPullServo.setPosition(baseServo.getPosition()+clampOffset-unclampedOffset);
        else armPullServo.setPosition(baseServo.getPosition()+clampOffset);
        Thread.sleep(300);
    }
    @Override
    public void run(SubsystemData data) throws InterruptedException {
        if(data.operatorGamepad.wasJustPressed(GamepadKeys.Button.X)) clamping=!clamping;
        if(clamping) armPullServo.setPosition(baseServo.getPosition()+clampOffset-unclampedOffset);
        else armPullServo.setPosition(baseServo.getPosition()+clampOffset);
    }

    public void update() throws InterruptedException {
        if(clamping) armPullServo.setPosition(baseServo.getPosition()+clampOffset-unclampedOffset);
        else armPullServo.setPosition(baseServo.getPosition()+clampOffset);
    }

    @Override
    public void initSubsystem(OpMode opMode) {
        super.initSubsystem(opMode);
        baseServo=hardwareMap.get(Servo.class, "swingBase");
        armPullServo=hardwareMap.get(Servo.class, "swingPullArm");
        armPullServo.setDirection(Servo.Direction.REVERSE);
        baseServo.setPosition(initialPosition);
        if(clamping) armPullServo.setPosition(baseServo.getPosition()+clampOffset-unclampedOffset);
        else armPullServo.setPosition(baseServo.getPosition()+clampOffset);
        clampButton=new ToggleButton(GamepadKeys.Button.X);
    }
}
