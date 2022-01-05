package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
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
        robot.initDrive(DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.FLOAT, new Pose2d(-31 , -62, Math.toRadians(180)));
        robot.logging.setLog("state", "Initialized, waiting for start");

        waitForStart();

        robot.logging.setLog("state", "Running");
        autonomousSequence();

        robot.stopRobot();
        robot.logging.setLog("state", "Stopped");
    }

    //autonomous sequence
    private void autonomousSequence() throws InterruptedException {

        TrajectorySequence trajectory = robot.drive.trajectorySequenceBuilder(new Pose2d(-31, -62, Math.toRadians(180)))
                .splineTo(new Vector2d(-62, -59), Math.toRadians(90))

                .addTemporalMarker(() -> robot.duckArm.moveArmBackward())
                .waitSeconds(3)
                .addTemporalMarker(() -> robot.duckArm.stopArm())

                .splineTo(new Vector2d(-62, -37), Math.toRadians(90))
                .build();
        
        robot.drive.followTrajectorySequence(trajectory);
    }
}
