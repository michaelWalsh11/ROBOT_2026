package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotBackground;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(name="El Horrible", group="Autonomous")
public final class TestOdometryAuto124 extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
//        Pose2d pose = new Pose2d(0, 0, 90);
//        MecanumDrive drive = new MecanumDrive(hardwareMap, pose);
        RobotBackground robot = new RobotBackground();
        robot.init(hardwareMap);
        AutoCommands go = new AutoCommands(hardwareMap);

        waitForStart();

        go.closeGrasper();
        go.graspAngler(0.25);
        go.armsPos(1100, 1000);
        robot.driveStraightInches(0.3, 35, 0.5);
        go.waitCommand(3000);
        //go.waitCommand(3000);
        go.armsPos(800, 900);
        go.openGrasper();
        go.waitCommand(1000);

        robot.driveStraightInches(0.5, -20, 0.5);

        go.waitCommand(2000);
    }
}
