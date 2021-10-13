package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareConfigIds;
import org.firstinspires.ftc.teamcode.R;

public class Arm extends RobotComponent{

    private Servo gripper;
    private DcMotor arm;

    //arm
    public int armPos;
    public int armMin = 0;
    public int armMax = 100;
    public double armSpeed;
    public int armDirection;

    //public bool armstate;

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
    public void gripperOpen (){
        gripper.setPosition(1);

    }

    public void gripperClose(){
        gripper.setPosition(0);
    }
    //the arm
    public void armUp(){
        armDirection = 1;
    }

    public void armDown (){
        armDirection = -1;

    }

    public void armStop(){
        armDirection = 0;
    }

    public void armLimits() throws InterruptedException {
        while(robot.isRunning){
            armPos = arm.getCurrentPosition();
            if (armPos >= armMax && armDirection == 1){
                armDirection = 0;
            }
            if (armPos >= armMin && armDirection == -1){
                armDirection = 0;
            }
            arm.setPower(armSpeed*armDirection);
            //tijd nog testem
            Thread.sleep(50);
        }
    }
}
