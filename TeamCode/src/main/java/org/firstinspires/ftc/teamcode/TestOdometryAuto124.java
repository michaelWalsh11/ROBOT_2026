package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.sql.Time;
import java.time.Duration;

@Autonomous(name="Odo Auto", group="Autonomous")
public final class TestOdometryAuto124 extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d pose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, pose);
        RobotBackground robot = new RobotBackground();

        waitForStart();

//        Actions.runBlocking(drive.actionBuilder(pose)
//                .lineToX(24)
//                .build());

        robot.armsMoveToVert(0.5, -1000);
        robot.armsMoveToPos(0.5, 1000);
        drive.actionBuilder(pose).lineToX(20);

        drive.actionBuilder(pose).lineToX(0);

    }
}
