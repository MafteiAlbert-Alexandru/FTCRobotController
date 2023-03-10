//package org.firstinspires.ftc.teamcode.subsystem;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.hardware.ToggleButton;
//
//@Config
//public class ClawSubsystem extends SmartSubsystem {
//    private Servo baseServo;
//    private ToggleButton clampButton;
//    private Servo armPullServo;
//    public static double position=0;
//    public static double initialPosition=0.35;
//    public static double ForwardPos = 0.04;
//    public static double BackwardPos = 0.59;
//    private boolean clamping = false;
//
//    public ClawSubsystem(){}
//
//    public ClawSubsystem(LinearOpMode opMode){
//        initSubsystem(opMode);
//    }
//
//    public boolean isClamping()
//    {return  clamping;
//    }
//    public void clamp()
//    {
//        clamping=true;
//        setPosition(position);
//    }
//
//    public void release()
//    {
//        clamping=false;
//        setPosition(position);
//    }
//    public void goTo(double position) throws InterruptedException {
//        double initialPosition = baseServo.getPosition();
//        double delta = Math.abs(position-initialPosition);
//        double time = delta * 300 / 200.0 *1000;
//        setPosition(position);
//        Thread.sleep((long) time);
//    }
//
//    public void setTo(double position) throws InterruptedException {
//        double initialPosition = baseServo.getPosition();
//        double delta = Math.abs(position-initialPosition);
//        double time = delta * 300 / 200.0 *1000;
//        setPosition(position);
//        Thread.sleep((long) time);
//    }
//
//    public void setPosition(double position)
//    {
//        this.position=position;
//
//        if(clamping) armPullServo.setPosition(position+clampOffset-unclampedOffset);
//        else armPullServo.setPosition(position+clampOffset);
//        baseServo.setPosition(position);
//    }
//    public void goToForward() throws InterruptedException {
//        baseServo.setPosition(ForwardPos);
//        if(clamping) armPullServo.setPosition(baseServo.getPosition()+clampOffset-unclampedOffset);
//        else armPullServo.setPosition(baseServo.getPosition()+clampOffset);
//    }
//    public void goToBackward() {
//        baseServo.setPosition(0.6);
//        if(clamping) armPullServo.setPosition(baseServo.getPosition()+clampOffset-unclampedOffset);
//        else armPullServo.setPosition(baseServo.getPosition()+clampOffset);
//    }
//    @Override
//    public void run(SubsystemData data) throws InterruptedException {
////        if(data.driverGamepad.wasJustPressed(GamepadKeys.Button.A)) clamping=!clamping;
////        if(clamping) armPullServo.setPosition(baseServo.getPosition()+clampOffset-unclampedOffset);
////        else armPullServo.setPosition(baseServo.getPosition()+clampOffset);
//    }
//
//    public void update() throws InterruptedException {
////        if(clamping) armPullServo.setPosition(baseServo.getPosition()+clampOffset-unclampedOffset);
////        else armPullServo.setPosition(baseServo.getPosition()+clampOffset);
//    }
//
//    @Override
//    public void initSubsystem(OpMode opMode) {
//        super.initSubsystem(opMode);
////        baseServo=hardwareMap.get(Servo.class, "swingBase");
////        armPullServo=hardwareMap.get(Servo.class, "swingPullArm");
////        armPullServo.setDirection(Servo.Direction.REVERSE);
////        baseServo.setPosition(initialPosition);
////        if(clamping) armPullServo.setPosition(baseServo.getPosition()+clampOffset-unclampedOffset);
////        else armPullServo.setPosition(baseServo.getPosition()+clampOffset);
////        clampButton=new ToggleButton(GamepadKeys.Button.X);
//    }
//}