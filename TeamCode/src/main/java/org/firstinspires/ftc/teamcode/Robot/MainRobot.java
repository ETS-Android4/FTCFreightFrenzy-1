package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class MainRobot {
    public Boolean isRunning = true;

    public SampleMecanumDrive drive; // roadrunner drive class
    public Gyroscope gyroscope;
    public Arm arm;
    public DuckArm DuckArm;
    public Logging logging;
    public ArrayList<RobotComponent> componentsList = new ArrayList<RobotComponent>();

    public LinearOpMode opMode;

    public MainRobot(HardwareMap hardwareMap, Telemetry inputTelemetry, String[] inputEnabledComponents, LinearOpMode inputOpmode) {
        opMode = inputOpmode;
        List<String> enabledComponents = Arrays.asList(inputEnabledComponents);

        /* always enabled components */
        logging = new Logging(inputTelemetry, this);
        componentsList.add(logging);

        drive = new SampleMecanumDrive(hardwareMap); // roadrunner drive class

        /* Switchable components */
        if(enabledComponents.contains("gyroscope")) {
            gyroscope = new Gyroscope(hardwareMap, this);
            componentsList.add(gyroscope);

        }
        if(enabledComponents.contains("arm")) {
            arm = new Arm(hardwareMap, this);
            componentsList.add(arm);
        }

        if(enabledComponents.contains("DuckArm")) {
            DuckArm = new DuckArm(hardwareMap, this);
            componentsList.add(DuckArm);
        }
    }

    public void initDrive(DcMotor.RunMode runMode, DcMotor.ZeroPowerBehavior zeroPowerBehavior, Pose2d startPose){
        drive.setMode(runMode);
        drive.setZeroPowerBehavior(zeroPowerBehavior);
        drive.setPoseEstimate(startPose);
    }

    public void checkForStop() throws InterruptedException{
        while(isRunning){
            if(opMode.isStopRequested())
                stopRobot();

            Thread.sleep(50);
        }
    }

    public void startThreads(){
        new Thread(() -> {
            try {
                checkForStop();
            } catch (InterruptedException ignored) { }
        }).start();

        for(int i = 0; i < componentsList.size(); i++){
            componentsList.get(i).startThreads();
        }
    }

    public void stopRobot(){
        isRunning = false;
    }
}
