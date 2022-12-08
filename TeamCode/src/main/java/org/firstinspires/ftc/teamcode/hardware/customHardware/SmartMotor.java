package org.firstinspires.ftc.teamcode.hardware.customHardware;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.HardwareDevice;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.function.Supplier;

public class SmartMotor implements HardwareDevice {

    public static double p = 0.006, i = 0.0001, d = 0.0005;
    public static double f = 0.1;
    public static double minPos, maxPos;
    public double ticks;
    final double ticks_in_degree = ticks / 180.0;
    public PIDController controller;
    public double PID_Target = 0;

    public void setPIDF(double p, double i, double d){
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    public enum GoBILDA {
        RPM_30(5264, 30), RPM_43(3892, 43), RPM_60(2786, 60), RPM_84(1993.6, 84),
        RPM_117(1425.2, 117), RPM_223(753.2, 223), RPM_312(537.6, 312), RPM_435(383.6, 435),
        RPM_1150(145.6, 1150), RPM_1620(103.6, 1620), BARE(28, 6000), NONE(0, 0);

        private double cpr, rpm;

        GoBILDA(double cpr, double rpm) {
            this.cpr = cpr;
            this.rpm = rpm;
        }

        public double getCPR() {
            return cpr;
        }

        public double getRPM() {
            return rpm;
        }

        public double getAchievableMaxTicksPerSecond() {
            return cpr * rpm / 60;
        }
    }

    public enum NeveRest {
        RPM_1780(103, 1780);

        private double cpr, rpm;

        NeveRest(double cpr, double rpm) {
            this.cpr = cpr;
            this.rpm = rpm;
        }

        public double getCPR() {
            return cpr;
        }

        public double getRPM() {
            return rpm;
        }

        public double getAchievableMaxTicksPerSecond() {
            return cpr * rpm / 60;
        }
    }

    public enum Direction {
        FORWARD(1), REVERSE(-1);

        private int val;

        Direction(int multiplier) {
            val = multiplier;
        }

        public int getMultiplier() {
            return val;
        }
    }

    public class Encoder {

        private Supplier<Integer> m_position;
        private int resetVal, lastPosition;
        private Direction direction;
        private double lastTimeStamp, veloEstimate, dpp, accel, lastVelo;

        public Encoder(Supplier<Integer> position) {
            m_position = position;
            dpp = 1;
            resetVal = 0;
            lastPosition = 0;
            veloEstimate = 0;
            direction = Direction.FORWARD;
            lastTimeStamp = (double) System.nanoTime() / 1E9;
        }

        public int getPosition() {
            int currentPosition = m_position.get();
            if (currentPosition != lastPosition) {
                double currentTime = (double) System.nanoTime() / 1E9;
                double dt = currentTime - lastTimeStamp;
                veloEstimate = (currentPosition - lastPosition) / dt;
                lastPosition = currentPosition;
                lastTimeStamp = currentTime;
            }
            return direction.getMultiplier() * currentPosition - resetVal;
        }


        public double getDistance() {
            return dpp * getPosition();
        }


        public double getRate() {
            return dpp * getVelocity();
        }

        public void reset() {
            resetVal += getPosition();
        }


        public Encoder setDistancePerPulse(double distancePerPulse) {
            dpp = distancePerPulse;
            return this;
        }


        public void setDirection(Direction direction) {
            this.direction = direction;
        }


        public double getRevolutions() {
            return getPosition() / getCPR();
        }


        public double getRawVelocity() {
            double velo = getVelocity();
            if (velo != lastVelo) {
                double currentTime = (double) System.nanoTime() / 1E9;
                double dt = currentTime - lastTimeStamp;
                accel = (velo - lastVelo) / dt;
                lastVelo = velo;
                lastTimeStamp = currentTime;
            }
            return velo;
        }


        public double getAcceleration() {
            return accel;
        }

        private final static int CPS_STEP = 0x10000;


        public double getCorrectedVelocity() {
            double real = getRawVelocity();
            while (Math.abs(veloEstimate - real) > CPS_STEP / 2.0) {
                real += Math.signum(veloEstimate - real) * CPS_STEP;
            }
            return real;
        }
    }

    public enum RunMode {
        VelocityControl, PositionControl, RawPower, PIDF_Arm
    }

    public enum ZeroPowerBehavior {
        UNKNOWN(DcMotor.ZeroPowerBehavior.UNKNOWN),
        BRAKE(DcMotor.ZeroPowerBehavior.BRAKE),
        FLOAT(DcMotor.ZeroPowerBehavior.FLOAT);

        private final DcMotor.ZeroPowerBehavior m_behavior;

        ZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
            m_behavior = behavior;
        }

        public DcMotor.ZeroPowerBehavior getBehavior() {
            return m_behavior;
        }
    }

    public DcMotor motor;
    public Encoder encoder;

    protected RunMode runmode;

    public double ACHIEVABLE_MAX_TICKS_PER_SECOND;


    protected GoBILDA type;

    protected PIDController veloController = new PIDController(1, 0, 0);

    protected PController positionController = new PController(1);

    protected SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 1, 0);

    private boolean targetIsSet = false;

    protected double bufferFraction = 0.9;

    public SmartMotor() {
    }

    public SmartMotor(@NonNull HardwareMap hMap, String id, MotorDirection type) {
        this(hMap, id, GoBILDA.NONE, type);
        ACHIEVABLE_MAX_TICKS_PER_SECOND = motor.getMotorType().getAchieveableMaxTicksPerSecond();
    }


    public enum MotorDirection {
        FORWARD, REVERSE
    }

    public MotorDirection Type;

    public SmartMotor(@NonNull HardwareMap hMap, String id, @NonNull GoBILDA gobildaType, MotorDirection type) {
        motor = hMap.get(DcMotor.class, id);
        encoder = new Encoder(motor::getCurrentPosition);

        runmode = RunMode.RawPower;
        if(type == MotorDirection.FORWARD) setInverted(false);
        else if(type == MotorDirection.REVERSE) setInverted(true);


        ACHIEVABLE_MAX_TICKS_PER_SECOND = gobildaType.getAchievableMaxTicksPerSecond();
    }

    public SmartMotor(@NonNull HardwareMap hMap, String id, @NonNull NeveRest gobildaType, MotorDirection type) {
        motor = hMap.get(DcMotor.class, id);
        encoder = new Encoder(motor::getCurrentPosition);

        runmode = RunMode.RawPower;
        if(type == MotorDirection.FORWARD) setInverted(false);
        else if(type == MotorDirection.REVERSE) setInverted(true);


        ACHIEVABLE_MAX_TICKS_PER_SECOND = gobildaType.getAchievableMaxTicksPerSecond();
    }


    public SmartMotor(@NonNull HardwareMap hMap, String id, double cpr, double rpm, MotorDirection Type) {
        this(hMap, id, GoBILDA.NONE, Type);

        MotorConfigurationType type = motor.getMotorType().clone();
        type.setMaxRPM(rpm);
        type.setTicksPerRev(cpr);
        motor.setMotorType(type);

        ACHIEVABLE_MAX_TICKS_PER_SECOND = cpr * rpm / 60;
    }

    public void set(double output) {
        if (runmode == RunMode.VelocityControl) {
            double speed = bufferFraction * output * ACHIEVABLE_MAX_TICKS_PER_SECOND;
            double velocity = veloController.calculate(getVelocity(), speed) + feedforward.calculate(speed, encoder.getAcceleration());
            motor.setPower(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND);
        } else if (runmode == RunMode.PositionControl) {
            double error = positionController.calculate(getDistance());
            motor.setPower(output * error);
        } else {
            motor.setPower(output);
        }
    }

    public void PIDF_Run(){
        if(runmode == RunMode.PIDF_Arm){

            controller.setPID(p, i, d);
            int armPos = encoder.getPosition();
            double pid = controller.calculate(armPos, PID_Target);
            double ff = Math.cos(Math.toRadians(PID_Target / ticks_in_degree)) * f;

            double power = pid + ff;

            motor.setPower(power);
        }
    }

    public Encoder setDistancePerPulse(double distancePerPulse) {
        return encoder.setDistancePerPulse(distancePerPulse);
    }


    public double getDistance() {
        return encoder.getDistance();
    }


    public double getRate() {
        return encoder.getRate();
    }

    public boolean atTargetPosition() {
        return positionController.atSetPoint();
    }

    public void resetEncoder() {
        encoder.reset();
    }

    public double[] getVeloCoefficients() {
        return veloController.getCoefficients();
    }

    public double getPositionCoefficient() {
        return positionController.getP();
    }

    public double[] getFeedforwardCoefficients() {
        return new double[]{feedforward.ks, feedforward.kv, feedforward.ka};
    }

    public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior.getBehavior());
    }

    public int getCurrentPosition() {
        return encoder.getPosition();
    }

    public double getCorrectedVelocity() {
        return encoder.getCorrectedVelocity();
    }

    public double getCPR() {
        return type == GoBILDA.NONE ? motor.getMotorType().getTicksPerRev() : type.getCPR();
    }

    public double getMaxRPM() {
        return type == GoBILDA.NONE ? motor.getMotorType().getMaxRPM() : type.getRPM();
    }

    public void setBuffer(double fraction) {
        if (fraction <= 0 || fraction > 1) {
            throw new IllegalArgumentException("Buffer must be between 0 and 1, exclusive to 0");
        }
        bufferFraction = fraction;
    }

    public void setRunMode(RunMode runmode) {
        this.runmode = runmode;
        veloController.reset();
        positionController.reset();
        if (runmode == RunMode.PositionControl && !targetIsSet) {
            setTargetPosition(getCurrentPosition());
            targetIsSet = false;
        }
    }

    protected double getVelocity() {
        return ((DcMotorEx) motor).getVelocity();
    }

    public double get() {
        return motor.getPower();
    }


    public void setTargetPosition(int target) {
        setTargetDistance(target * encoder.dpp);
    }


    public void setTargetDistance(double target) {
        targetIsSet = true;
        positionController.setSetPoint(target);
    }

    public void setPositionTolerance(double tolerance) {
        positionController.setTolerance(tolerance);
    }


    public void setInverted(boolean isInverted) {
        motor.setDirection(isInverted ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
    }


    public boolean getInverted() {
        return DcMotor.Direction.REVERSE == motor.getDirection();
    }


    public void setVeloCoefficients(double kp, double ki, double kd) {
        veloController.setPIDF(kp, ki, kd, 0);
    }

    public void setFeedforwardCoefficients(double ks, double kv) {
        feedforward = new SimpleMotorFeedforward(ks, kv);
    }


    public void setFeedforwardCoefficients(double ks, double kv, double ka) {
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);
    }

    public void setPositionCoefficient(double kp) {
        positionController.setP(kp);
    }

    @Override
    public void disable() {
        motor.close();
    }

    @Override
    public String getDeviceType() {
        return "Motor " + motor.getDeviceName() + " from " + motor.getManufacturer()
                + " in port " + motor.getPortNumber();
    }

    public void stopMotor() {
        motor.setPower(0);
    }

}
