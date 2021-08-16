package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.TreeMap;

public class Logging extends RobotComponent {
    //hardwareMap and telemetry
    private final Telemetry telemetry;

    private TreeMap<String, Object> logs = new TreeMap<String, Object>();
    private int updateDelay = 100;

    public Logging(Telemetry inputTelemetry, MainRobot inputRobot) {
        super(inputRobot);

        telemetry = inputTelemetry;
    }

    @Override
    public void startThreads(){
        new Thread(() -> {
            try {
                updateLogs();
            } catch (InterruptedException ignored) { }
        }).start();
    }

    public void setLog(String key, Object value){
        formatSetLog(key, value);
    }
    public void removeLog(String key){
        logs.remove(key);
    }

    public void clearLogs(){
        logs.clear();
    }

    private void formatSetLog(String key, Object value){
        logs.put(key, value);
    }

    public void updateLogs() throws InterruptedException {
        while (robot.isRunning){
            for(HashMap.Entry<String, Object> entry : logs.entrySet()){
                telemetry.addData(entry.getKey(), entry.getValue());
            }
            telemetry.update();

            Thread.sleep(updateDelay);
        }
    }
}
