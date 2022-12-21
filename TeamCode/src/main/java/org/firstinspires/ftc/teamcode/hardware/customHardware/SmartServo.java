//package org.firstinspires.ftc.teamcode.hardware.customHardware;
//
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.ServoImpl;
//import com.qualcomm.robotcore.util.Range;
//
//public class SmartServo extends ServoImpl {
//
//
//
//    public enum SmartServoType {
//        GOBILDA_TORQUE(8.333333333333334, 300),
//        GOBILDA_SPEED(18.75, 300),
//        GOBILDA_SUPERSPEED(37.5, 300);
////        GOBILDA_TORQUE_5TURN,
////        GOBILDA_SPEED_5TURN,
////        GOBILDA_SUPERSPEED_5TURN;
//
//        private double speedCoefficient;
//        SmartServoType(double voltageCoefficient, double range)
//        {
//            this.speedCoefficient=range/voltageCoefficient*1000.0;
//        }
//
//        public double getSpeedCoefficient(double voltage)
//        {
//            return speedCoefficient/voltage;
//        }
//
//    };
//    private double voltage;
//    private SmartServoType servoType;
//    private Double lastPosition = null;
//    private Long lastTime = null;
//    private Double lastTarget = null;
//
//    public SmartServo(Servo servo, SmartServoType type)
//    {
//        this(servo, type, 4.8);
//    }
//    public SmartServo(Servo servo, SmartServoType type, double voltage)
//    {
//        super(servo.getController(), servo.getPortNumber());
//        servoType=type;
//        this.voltage=voltage;
//    }
//
//
//
//    /**
//     * Get the position of the servo
//     * @param trackedPosition is true, it returns actual position based on speed calculations
//     * @return position of the servo
//     */
//    public double getPosition(boolean trackedPosition)
//    {
//        if(trackedPosition&&lastTarget!=null) {
//
//            // x=v*t
//            double deltaTime = (System.nanoTime()-lastTime);
//
//            double position = lastPosition+Math.signum(lastTarget-lastPosition)*Math.min(Math.abs(lastTarget-lastPosition), servoType.getSpeedCoefficient(voltage)*deltaTime);
//            if(getDirection()==Direction.REVERSE)position=reverse(position);
//            return Range.scale(position, limitPositionMin, limitPositionMax, MIN_POSITION, MAX_POSITION);
//        }else {
//            double position = controller.getServoPosition(getPortNumber());
//            if(getDirection()==Direction.REVERSE)position=reverse(position);
//            return Range.scale(position, limitPositionMin, limitPositionMax, MIN_POSITION, MAX_POSITION);
//        }
//    }
//
//    /**
//     * Sets the position of the servo
//     * @param position position of the servo
//     * @param blocking if this command should block
//     */
//    public void setPosition(double position, boolean blocking)
//    {
//        // Do position scaling, using scale set by setRange
//        position = Range.scale(position, MIN_POSITION, MAX_POSITION, limitPositionMin, limitPositionMax);
//        if(getDirection()==Direction.REVERSE) position=reverse(position);
//
//        // Update tracking parameters
//        lastPosition = getPosition(true);
//        lastTime = System.nanoTime();
//        lastTarget = position;
//
//
//        controller.setServoPosition(getPortNumber(), position);
//
//        if(blocking) {
//            try { // Blocking sleep
//                Thread.sleep((long) (servoType.getSpeedCoefficient(voltage)*Math.abs(lastTarget-lastPosition)));
//            }catch (Exception e) { e.printStackTrace(); }
//        }
//
//
//    }
//
//    private double reverse(double position) {
//        return MAX_POSITION - position + MIN_POSITION;
//    }
//
//    @Override
//    public void setPosition(double position) {
//        this.setPosition(position, false);
//    }
//
//    @Override
//    public double getPosition() {
//        return this.getPosition(false);
//    }
//}
