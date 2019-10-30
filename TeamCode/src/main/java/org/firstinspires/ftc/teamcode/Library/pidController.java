package org.firstinspires.ftc.teamcode.Library;

public class pidController {

    private long lastTime;
    private double Input, Output, Setpoint;
    private double ITerm, lastInput;
    private double kp, ki, kd;
    private long SampleTime = 1000; //1 sec
    private double outMin, outMax;
    private boolean inAuto = false;

    public final int MANUAL=0;
    public final int AUTOMATIC=1;

    public final int DIRECT=0;
    public final int REVERSE=1;

    private int controllerDirection = DIRECT;

    public void pidController() {
    }

    public void pidController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;

        SetTunings(kp, ki, kd);
    }

    public double Compute() {
        if(!inAuto) return 0.0;

        long now = System.nanoTime() / 1000;
        long timeChange = (now - lastTime);

        if (timeChange>=SampleTime) {
            /*Compute all the working error variables*/
            double error = Setpoint - Input;
            ITerm+= (ki * error);
            if (ITerm > outMax) ITerm= outMax;
            else if(ITerm < outMin) ITerm= outMin;
            double dInput = (Input - lastInput);

            /*Compute PID Output*/
            Output = kp * error + ITerm- kd * dInput;
            if (Output > outMax) Output = outMax;
            else if (Output < outMin) Output = outMin;

            /*Remember some variables for next time*/
            lastInput = Input;
            lastTime = now;
        }

        return Output;
    }

    public double getOutput() {
        return Output;
    }

    public void SetTunings(double Kp, double Ki, double Kd) {
        if (Kp<0 || Ki<0|| Kd<0) return;

        double SampleTimeInSec = ((double)SampleTime)/1000;
        kp = Kp;
        ki = Ki * SampleTimeInSec;
        kd = Kd / SampleTimeInSec;

        if (controllerDirection == REVERSE) {
            kp = (0 - kp);
            ki = (0 - ki);
            kd = (0 - kd);
        }
    }

    public void SetSampleTime(long NewSampleTime) {
        if (NewSampleTime > 0) {
            double ratio  = (double)NewSampleTime
                    / (double)SampleTime;
            ki *= ratio;
            kd /= ratio;
            SampleTime = (long)NewSampleTime;
        }
    }

    public void SetOutputLimits(double Min, double Max) {
        if (Min > Max) return;
        outMin = Min;
        outMax = Max;

        if (Output > outMax) Output = outMax;
        else if (Output < outMin) Output = outMin;

        if (ITerm > outMax) ITerm = outMax;
        else if (ITerm < outMin) ITerm = outMin;
    }

    public void SetMode(int Mode) {
        boolean newAuto = (Mode == AUTOMATIC);
        if (newAuto == !inAuto) {  /*we just went from manual to auto*/
            Initialize();
        }
        inAuto = newAuto;
    }

    public void Initialize() {
        lastInput = Input;
        ITerm = Output;
        if (ITerm > outMax) ITerm = outMax;
        else if (ITerm < outMin) ITerm = outMin;
    }

    public void SetControllerDirection(int Direction) {
        controllerDirection = Direction;
    }
}

