package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Robot.MainRobot;

@Autonomous(name="TeamBlue_ToDuck", group="")
public class TeamBlue_ToDuck extends LinearOpMode {
    private MainRobot robot;

    public Pose2d startPose = new Pose2d(-31, 62, Math.toRadians(0));

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
    private void autonomousSequence() {
        TrajectorySequence trajectory = robot.drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(-62, 56), Math.toRadians(180))
                .setReversed(false)

                .addTemporalMarker(() -> robot.duckArm.moveArmForward())
                .waitSeconds(5)
                .addTemporalMarker(() -> robot.duckArm.stopArm())

                .turn(Math.toRadians(-90))
                .splineTo(new Vector2d(-65, 37), Math.toRadians(-90))
                .build();

        robot.drive.followTrajectorySequence(trajectory);
    }
}
