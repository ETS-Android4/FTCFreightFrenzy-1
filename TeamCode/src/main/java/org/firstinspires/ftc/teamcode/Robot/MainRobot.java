package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Disabled
public class MainRobot {
    public Boolean isRunning = true;

//    public Driving driving;
    public SampleMecanumDrive drive;
    public Gyroscope gyroscope;
    public Logging logging;
    public ArrayList<RobotComponent> componentsList = new ArrayList<RobotComponent>();

    public LinearOpMode opMode;

    public MainRobot(HardwareMap hardwareMap, Telemetry inputTelemetry, String[] inputEnabledComponents, LinearOpMode inputOpmode) {
        opMode = inputOpmode;

        List<String> enabledComponents = Arrays.asList(inputEnabledComponents);

        //always enabled components
        logging = new Logging(inputTelemetry, this);
        componentsList.add(logging);

        gyroscope = new Gyroscope(hardwareMap, this);
        componentsList.add(gyroscope);

//        driving = new Driving(hardwareMap, this);
//        componentsList.add(driving);
        drive = new SampleMecanumDrive(hardwareMap);

        //Switchable components
//        if(enabledComponents.contains("ComponentName")) {
//            component = new Component(inputTelemetry, this);
//            componentsList.add(component);
//        }
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
