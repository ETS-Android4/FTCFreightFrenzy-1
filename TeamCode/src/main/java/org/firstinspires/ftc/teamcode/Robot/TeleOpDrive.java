package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareConfigIds;

public class TeleOpDrive extends RobotComponent{
    private final DcMotor wheelLF;
    private final DcMotor wheelRF;
    private final DcMotor wheelRB;
    private final DcMotor wheelLB;

    public TeleOpDrive(HardwareMap hardwareMap, MainRobot inputRobot) {
        super(inputRobot);

        wheelLF = hardwareMap.get(DcMotor.class, HardwareConfigIds.motorLeftFront);
        wheelRF = hardwareMap.get(DcMotor.class, HardwareConfigIds.motorRightFront);
        wheelRB = hardwareMap.get(DcMotor.class, HardwareConfigIds.motorRightBack);
        wheelLB = hardwareMap.get(DcMotor.class, HardwareConfigIds.motorLeftBack);

        wheelLF.setDirection(DcMotor.Direction.REVERSE);
        wheelLB.setDirection(DcMotor.Direction.REVERSE);

        setWheelRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setWheelZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void startThreads(){

    }

    public void setWheelRunMode(DcMotor.RunMode mode){
        wheelLF.setMode(mode);
        wheelRF.setMode(mode);
        wheelRB.setMode(mode);
        wheelLB.setMode(mode);
    }
    public void setWheelZeroPowerBehaviour(DcMotor.ZeroPowerBehavior zeroPowerBehaviour){
        wheelLF.setZeroPowerBehavior(zeroPowerBehaviour);
        wheelRF.setZeroPowerBehavior(zeroPowerBehaviour);
        wheelRB.setZeroPowerBehavior(zeroPowerBehaviour);
        wheelLB.setZeroPowerBehavior(zeroPowerBehaviour);
    }

    public void setWheelPowers(double[] wheelPowers){
        wheelLF.setPower(wheelPowers[0]);
        wheelRF.setPower(wheelPowers[1]);
        wheelRB.setPower(wheelPowers[2]);
        wheelLB.setPower(wheelPowers[3]);
    }
}
