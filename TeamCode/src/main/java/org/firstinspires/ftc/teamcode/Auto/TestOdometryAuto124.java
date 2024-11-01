package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotBackground;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(name="Odo Auto Hang 1 try hope", group="Autonomous")
public final class TestOdometryAuto124 extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d pose = new Pose2d(0, 0, 90);
        MecanumDrive drive = new MecanumDrive(hardwareMap, pose);
        RobotBackground robot = new RobotBackground();
        AutoCommands go = new AutoCommands(hardwareMap);

        waitForStart();

//        go.closeGrasper();
//        go.graspAngler(0.25);
//        go.armsPos(1000, 1000);
//
//        go.waitCommand(3000);

//        Actions.runBlocking(
//                drive.actionBuilder(pose)
//                        .splineTo(new Vector2d(30, 0), 0)
//                        .build());

        Actions.runBlocking(
                drive.actionBuilder(pose)
                        .strafeTo(new Vector2d(0, -30))
                        .build());


        //pose = drive.pose;

//        go.waitCommand(3000);
//        go.armsPos(800, 900);
//        go.openGrasper();
//        go.waitCommand(1000);

//        Actions.runBlocking(
//                drive.actionBuilder(pose)
//                        .splineTo(new Vector2d(0, 0), Math.toRadians(0))
//                        .build());
//
//        pose = drive.pose;

        go.waitCommand(2000);
    }
}
