package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareConfigIds;

public class DuckArm extends RobotComponent{
    private DcMotor duckArm;

    public DuckArm(HardwareMap hardwareMap, MainRobot inputRobot) {
        super(inputRobot);

        //initializing method
        duckArm = hardwareMap.get(DcMotor.class, HardwareConfigIds.duckArm);
    }

    @Override
    public void startThreads(){
        //start threads here if necessary
    }

    public void moveArmForward (){
        duckArm.setPower(1);
    }
    public void moveArmBackward(){
        duckArm.setPower(-1);
    }

    public void stopArm (){
        duckArm.setPower(0);
    }
}