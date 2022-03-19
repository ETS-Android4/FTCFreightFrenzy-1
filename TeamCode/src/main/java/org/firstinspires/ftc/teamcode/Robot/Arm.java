package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareConfigIds;
import org.firstinspires.ftc.teamcode.Misc.MathFunctions;
import org.firstinspires.ftc.teamcode.R;

public class Arm extends RobotComponent{
    private Servo gripper;
    private DcMotor arm;

    //arm
    public double armMin = 0;
        public double armMax = 1350;

    public Arm(HardwareMap hardwareMap, MainRobot inputRobot) {
        super(inputRobot);

        //initializing method
        arm = hardwareMap.get(DcMotor.class, HardwareConfigIds.arm);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gripper = hardwareMap.get(Servo.class, HardwareConfigIds.gripper);
        gripper.setDirection(Servo.Direction.REVERSE);
        gripper.scaleRange(0.7, 1.0);// was 0,2 0,8

    }

    @Override
    public void startThreads(){

    }

    //the gripper
    public void gripperOpen (){
        gripper.setPosition(1);
    }
    public void gripperClose(){
        gripper.setPosition(0);
    }

    //the arm commands
    public void armUp(){
        if (arm.getCurrentPosition() <= armMax){
            arm.setPower(0.3);
        } else{
            arm.setPower(0);
        }
    }

    public void armDown (){
        if (arm.getCurrentPosition() >= armMin){
            arm.setPower(-0.3);
        } else {
            arm.setPower(0);
        }
    }

    public void stopArm (){
        arm.setPower(0);
        robot.logging.setLog("arm position", arm.getCurrentPosition());
    }


    public void armToPos(int pos) {
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int posDif = Math.abs(arm.getCurrentPosition()-pos);
        while(posDif > 100){
            try{
                Thread.sleep(100);
            } catch (InterruptedException e) {}

            posDif = Math.abs(arm.getCurrentPosition()-pos);
        }

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
