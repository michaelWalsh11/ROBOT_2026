package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="This will almost defiantly not work but ig ill try", group="RR")
public class RoadrunnerDrive extends LinearOpMode
{

    private double DRIVE_SPEED = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {




        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            RobotBackground robot = new RobotBackground();

            robot.init(hardwareMap);

            robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                double leftX;
                double leftY;
                double rightX;

                leftX = gamepad1.left_stick_x * DRIVE_SPEED;
                leftY = gamepad1.left_stick_y * DRIVE_SPEED;
                rightX = gamepad1.right_stick_x * DRIVE_SPEED;

                double leftRearPower = leftY + leftX - rightX;
                double leftFrontPower = leftY - leftX - rightX;
                double rightRearPower = leftY - leftX + rightX;
                double rightFrontPower = leftY + leftX + rightX;

                robot.leftFront.setPower(leftFrontPower);
                robot.leftRear.setPower(leftRearPower);
                robot.rightFront.setPower(rightFrontPower);
                robot.rightRear.setPower(rightRearPower);



                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        }
    }
}
