package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareConfigIds;
import org.firstinspires.ftc.teamcode.R;

public class Arm extends RobotComponent{

    private Servo gripper;
    private DcMotor arm;

    //public bool armstate;

    public Arm(HardwareMap hardwareMap, MainRobot inputRobot) {
        super(inputRobot);

        //initializing method
            gripper = hardwareMap.get(Servo.class, HardwareConfigIds.gripper);
            arm = hardwareMap.get(DcMotor.class, HardwareConfigIds.arm);

        //test = hardwareMap.get(Servo.class, R.string.);
    }

    @Override
    public void startThreads(){
        //start threads here if necessary
    }
    //the gripper
    public void gripperOpen (){

    }

    public void gripperClose(){

    }
    //the arm
    public void armUp(){

    }

    public void armDown (){

    }
}
