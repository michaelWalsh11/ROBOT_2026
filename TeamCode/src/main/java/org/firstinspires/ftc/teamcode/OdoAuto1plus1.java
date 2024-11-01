
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Odo Auto put it on and put  it in (bucket) - Hunter Nguyen a.k.a some bs", group="Autonomous")
public final class OdoAuto1plus1 extends LinearOpMode
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

        go.armsPos(0, 0);

        Actions.runBlocking(drive.actionBuilder(pose)
                .splineTo(new Vector2d(0, 24), Math.toRadians(90))
                .build());

        pose = drive.pose;

        go.armsPos(4000, 1400);

        Actions.runBlocking(drive.actionBuilder(pose)
                .lineToY(30)
                .build());

        pose = drive.pose;

        go.waitCommand(3000);
        go.openGrasper();
        go.waitCommand(1000);

        Actions.runBlocking(drive.actionBuilder(pose)
                .lineToY(24)
                .build());

        pose = drive.pose;

        go.vertArmToPos(0);
        go.waitCommand(1000);
        go.horArmToPos(0);

        go.waitCommand(3000);

    }
}
