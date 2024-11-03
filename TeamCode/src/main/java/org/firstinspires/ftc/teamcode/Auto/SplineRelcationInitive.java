package org.firstinspires.ftc.teamcode.Auto;

import android.app.Notification;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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

            //RobotBackground robot = new RobotBackground();
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            AutoCommands go = new AutoCommands(hardwareMap);

            TrajectoryActionBuilder tab1 = drive.actionBuilder(beginPose)
                    .lineToYConstantHeading(20);

            TrajectoryActionBuilder tab2 = drive.actionBuilder(beginPose)
                    .lineToYConstantHeading(10);

            waitForStart();

            Actions.runBlocking(
                    new SequentialAction(
                            tab1.build(),
                            go.closeClaw(),
                            go.vertArmToPos(500),
                            tab2.build()
                    )
            );

            while (opModeIsActive())
            {

            }

        } else {
            throw new RuntimeException();
        }
    }
}
