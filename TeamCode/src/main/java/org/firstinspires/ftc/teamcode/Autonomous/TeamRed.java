package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot.MainRobot;

@Autonomous(name="TeamRed", group="")
public class TeamRed extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        String[] enabledComponents = {"duckArm"};
        robot = new MainRobot(hardwareMap, telemetry, enabledComponents, this);

        robot.logging.setLog("state", "Initializing");
        robot.startThreads();
        robot.initDrive(DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.FLOAT, new Pose2d(102 , 136, Math.toRadians(-90)));
        robot.logging.setLog("state", "Initialized, waiting for start");

        waitForStart();

        robot.logging.setLog("state", "Running");
        autonomousSequence();

        robot.stopRobot();
        robot.logging.setLog("state", "Stopped");
    }

    //autonomous sequence
    private void autonomousSequence() throws InterruptedException {

        Trajectory traj1 = robot.drive.trajectoryBuilder(new Pose2d(102, 136, Math.toRadians(-90)))
                .splineTo(new Vector2d(130, 130), Math.toRadians(-90))
                .build();


        Trajectory traj2 = robot.drive.trajectoryBuilder(new Pose2d(130, 130, Math.toRadians(-90)))
                .splineTo(new Vector2d(136, 100), Math.toRadians(-90))
                .build();
        
        robot.drive.followTrajectory(traj1);
        robot.duckArm.moveArm();
        Thread.sleep(1000);
        robot.duckArm.stopArm();
        robot.drive.followTrajectory(traj2);
    }

    /*
    AutonomousDuckArm();
     */
}
