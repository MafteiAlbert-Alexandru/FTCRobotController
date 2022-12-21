//package org.firstinspires.ftc.teamcode.hardware.customHardware;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//public class ServoMirror{
//
//    private Servo LeftServo;
//    private Servo RightServo;
//
//    public ServoMirror(Servo left, Servo right){
//        LeftServo = left;
//        RightServo = right;
//        RightServo.setDirection(Servo.Direction.REVERSE);
//        LeftServo.setDirection(Servo.Direction.FORWARD);
//    }
//
//    public ServoMirror(HardwareMap hardwareMap, String leftServoName, String rightServoName){
//        LeftServo = hardwareMap.get(Servo.class, leftServoName);
//        RightServo = hardwareMap.get(Servo.class, rightServoName);
//        RightServo.setDirection(Servo.Direction.REVERSE);
//        LeftServo.setDirection(Servo.Direction.FORWARD);
//    }
//
//    public void setPosition(double pos){
//        if(pos>1) pos = 1;
//        else if(pos<0) pos = 0;
//
//        LeftServo.setPosition(pos);
//        RightServo.setPosition(pos);
//    }
//
//    public int getPosition(){
//        return (int)(LeftServo.getPosition() + RightServo.getPosition())/2;
//    }
//
//    public void addPosition(double addedPos){
//        setPosition(getPosition() + addedPos);
//    }
//}
