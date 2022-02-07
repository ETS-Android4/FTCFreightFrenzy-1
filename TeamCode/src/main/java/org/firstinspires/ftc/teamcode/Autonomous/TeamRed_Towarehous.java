package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Robot.MainRobot;

@Autonomous(name="TeamRed_ToWarehous", group="")
public class TeamRed_ToWarehous extends LinearOpMode {
    private MainRobot robot;

    public Pose2d startPose = new Pose2d(-15, -69, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException{
        String[] enabledComponents = {"duckArm"};
        robot = new MainRobot(hardwareMap, telemetry, enabledComponents, this);

        robot.logging.setLog("state", "Initializing");
        robot.startThreads();
        robot.initDrive(DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.FLOAT, startPose);
        robot.logging.setLog("state", "Initialized, waiting for start");

        waitForStart();

        robot.logging.setLog("state", "Running");
        autonomousSequence();

        robot.stopRobot();
        robot.logging.setLog("state", "Stopped");
    }

    //autonomous sequence
    private void autonomousSequence() throws InterruptedException {
        //this is the sequense for moving to the warehuis
        TrajectorySequence trajectory = robot.drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(12, -46), Math.toRadians(0))
                //hij rijdt naar positie en draait
                .splineTo(new Vector2d(47,-46), Math.toRadians(0))

                .build();
        robot.drive.followTrajectorySequence(trajectory);
    }
}
