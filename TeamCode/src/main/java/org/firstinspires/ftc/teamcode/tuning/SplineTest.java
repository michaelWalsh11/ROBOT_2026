package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 90);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            TrajectoryActionBuilder tab1 = drive.actionBuilder(beginPose)
                    .lineToYConstantHeading(20);

            TrajectoryActionBuilder tab2 = drive.actionBuilder(beginPose)
                    .lineToYConstantHeading(10);

            waitForStart();

            Actions.runBlocking(
                    new SequentialAction(
                            tab1.build(),
                            tab2.build()
                    ));



//            Actions.runBlocking(
//                    drive.actionBuilder(beginPose)
//                            //.strafeTo(new Vector2d(0, 30))
//                            .waitSeconds(5)
//                            .lineToYConstantHeading(20)
//                            .lineToYConstantHeading(0)
//                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}