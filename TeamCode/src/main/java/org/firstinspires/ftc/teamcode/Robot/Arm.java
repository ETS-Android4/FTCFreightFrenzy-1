package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareConfigIds;
import org.firstinspires.ftc.teamcode.Misc.MathFunctions;
import org.firstinspires.ftc.teamcode.R;

public class Arm extends RobotComponent{
    private Servo gripper;
    private DcMotor arm;

    //arm
    public int armPos;
    public double armMin = 0;
    public double armMax = 100;
    public double armSlowRange = 50;
    public double armSpeed;
    public int armDirection;

    public Arm(HardwareMap hardwareMap, MainRobot inputRobot) {
        super(inputRobot);

        //initializing method
        gripper = hardwareMap.get(Servo.class, HardwareConfigIds.gripper);
        arm = hardwareMap.get(DcMotor.class, HardwareConfigIds.arm);
    }

    @Override
    public void startThreads(){
        //start threads here if necessary
        new Thread(){
            @Override
            public void run(){
                try {
                    armLimits();
                } catch (Exception ignored) { }
            }
        }.start();
    }
    //the gripper
    public void gripperOpen (){ gripper.setPosition(1);}
    public void gripperClose(){
        gripper.setPosition(0);
    }

    //the arm commands
    public void armUp(){
        armDirection = 1;
    }

    public void armDown (){ armDirection = -1;}

    public void armStop(){ armDirection = 0;}

    public void armLimits() throws InterruptedException {
        while (robot.isRunning) {
            armPos = arm.getCurrentPosition();

            double power = armSpeed*armDirection;
            if(armPos <= armMin+armSlowRange && armDirection == -1)
                    power *= Math.max(0, (armPos-armMin)/armSlowRange);
            else if(armPos >= armMax-armSlowRange && armDirection == 1)
                    power *= Math.max(0, (armMax-armPos)/armSlowRange);

            arm.setPower(power);

            Thread.sleep(50);
        }
    }
}
