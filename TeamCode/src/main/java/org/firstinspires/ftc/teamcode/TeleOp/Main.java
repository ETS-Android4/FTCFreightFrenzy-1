package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot.MainRobot;

@Disabled
@TeleOp(name = "Main", group = "")
public class Main extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode () throws InterruptedException{
        String[] enabledComponents = { };
        robot = new MainRobot(hardwareMap, telemetry, enabledComponents, this);

        robot.logging.setLog("state", "Initializing");
        robot.startThreads();
        robot.initDrive(DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.FLOAT, new Pose2d(0, 0, 0));
        robot.logging.setLog("state", "Initialized, waiting for start");

        waitForStart();

        robot.logging.setLog("state", "Running");
        controlLoop();

        robot.stopRobot();
        robot.logging.setLog("state", "Stopped");
    }

    private void controlLoop() {
        while (opModeIsActive()){
            Drive();
            arm();

        }
    }
    public void arm(){

         if (gamepad1.dpad_up){
            robot.arm.armUp();
         }
         if (gamepad2.dpad_down){
            robot.arm.armDown();
         }
        if (gamepad1.dpad_left){
            robot.arm.gripperOpen();
        }
        if (gamepad2.dpad_right){
            robot.arm.gripperClose();
        }
    }
    public void Drive(){
        //joystick
        double LY = -1 * gamepad1.left_stick_y;
        double LX = gamepad1.left_stick_x;
        double RX = gamepad1.right_stick_x;

        //algemene formule
        double LAchter = LY - LX + RX;
        double RAchter = LY + LX - RX;
        double LVoor = LY + LX + RX;
        double RVoor = LY - LX - RX;
        //let op volgorede testen
        robot.drive.setMotorPowers(LVoor, LAchter, RVoor, RAchter);
    }
}
