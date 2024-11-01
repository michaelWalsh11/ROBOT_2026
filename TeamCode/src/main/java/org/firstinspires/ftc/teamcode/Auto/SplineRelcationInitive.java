package org.firstinspires.ftc.teamcode.Auto;

import android.app.Notification;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotBackground;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(name="Spline Test Test 2", group="Autonomous")
public final class SplineRelcationInitive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 90);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            RobotBackground robot = new RobotBackground();

            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            AutoCommands go = new AutoCommands(hardwareMap);

            //Trajectory one = drive.actionBuilder(beginPose).lineToYConstantHeading(20, 0);
            waitForStart();

            //go.closeGrasper();
            drive.actionBuilder(beginPose)
                    .lineToYConstantHeading(20)
                    //.lineToYConstantHeading(0)
                    .build();

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActionChosen,
                            go.openGrasper(),
                            go.armsPos(1000, 1000),
                            trajectoryActionCloseOut
                    )
            );

            go.armsPos(1000, 1000);
//
            //go.waitCommand(3000);
            while (opModeIsActive())
            {

            }

        } else {
            throw new RuntimeException();
        }
    }
}