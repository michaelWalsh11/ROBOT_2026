package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Odo Auto Hang straight (pray)", group="Autonomous")
public final class TestOdometryAuto124 extends LinearOpMode
{
    @Override
    public void runOpMode() {
        Pose2d pose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, pose);
        RobotBackground robot = new RobotBackground();
        AutoCommands go = new AutoCommands();

        waitForStart();

        go.closeGrasper();
        go.armsPos(1000, 1000);

        Actions.runBlocking(drive.actionBuilder(pose)
                .lineToX(24)
                .build());

        pose = drive.pose;

        go.waitCommand(3000);
        go.armsPos(800, 900);
        go.openGrasper();
        go.waitCommand(1000);

        Actions.runBlocking(drive.actionBuilder(pose)
                .lineToX(0)
                .build());

        pose = drive.pose;

        go.waitCommand(2000);
    }
}
