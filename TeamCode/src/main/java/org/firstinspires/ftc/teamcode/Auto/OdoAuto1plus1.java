
package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="Odo Auto put it on and put  it in (bucket) - Hunter Nguyen a.k.a some bs", group="Autonomous")
public final class OdoAuto1plus1 extends LinearOpMode
{
    @Override
    public void runOpMode() {
        Pose2d pose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, pose);
        AutoCommands go = new AutoCommands(hardwareMap);

        waitForStart();

        go.closeGrasper();
        go.armsPos(1000, 1000);

        Actions.runBlocking(drive.actionBuilder(pose)
                .lineToXConstantHeading(24)
                .build());

        pose = drive.pose;

        go.waitCommand(3000);
        go.armsPos(800, 900);
        go.openGrasper();
        go.waitCommand(1000);

        go.armsPos(0, 0);

        Actions.runBlocking(drive.actionBuilder(pose)
                .lineToX(20)
                .splineTo(new Vector2d(12, 24), Math.toRadians(90))
                .build());

        pose = drive.pose;

        go.closeGrasper();
        go.armsPos(4000, 1400);

        Actions.runBlocking(drive.actionBuilder(pose)
                .lineToXConstantHeading(0)
                .lineToYConstantHeading(30)
                .build());

        pose = drive.pose;

        go.waitCommand(3000);
        go.openGrasper();
        go.waitCommand(1000);

        Actions.runBlocking(drive.actionBuilder(pose)
                .lineToYConstantHeading(24)
                .build());

        pose = drive.pose;

        go.vertArmToPos(0);
        go.waitCommand(1000);
        go.horArmToPos(0);

        go.waitCommand(3000);

    }
}
