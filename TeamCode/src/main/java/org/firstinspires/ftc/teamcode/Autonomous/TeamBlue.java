package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Robot.MainRobot;

@Autonomous(name="TeamBlue", group="")
public class TeamBlue extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        String[] enabledComponents = {"duckArm"};
        robot = new MainRobot(hardwareMap, telemetry, enabledComponents, this);

        robot.logging.setLog("state", "Initializing");
        robot.startThreads();
        robot.initDrive(DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.FLOAT, new Pose2d(-31, 62, Math.toRadians(0)));
        robot.logging.setLog("state", "Initialized, waiting for start");

        waitForStart();

        robot.logging.setLog("state", "Running");
        autonomousSequence();

        robot.stopRobot();
        robot.logging.setLog("state", "Stopped");
    }

    //autonomous sequence
    private void autonomousSequence() throws InterruptedException {
        TrajectorySequence traj1 = robot.drive.trajectorySequenceBuilder(new Pose2d(-31, 62, Math.toRadians(0)))
                .setReversed(true)
                .splineTo(new Vector2d(-57, 59), Math.toRadians(180))
                .setReversed(false)

                .addTemporalMarker(() -> robot.duckArm.moveArm())
                .waitSeconds(3)
                .addTemporalMarker(() -> robot.duckArm.stopArm())

                .turn(Math.toRadians(-120))
                //.strafeTo(new Vector2d(-63, 35))
                .splineTo(new Vector2d(-73, 35), Math.toRadians(-120))
                .build();

        robot.drive.followTrajectorySequence(traj1);
    }

    /*
    AutonomousDuckArm();
     */
}
