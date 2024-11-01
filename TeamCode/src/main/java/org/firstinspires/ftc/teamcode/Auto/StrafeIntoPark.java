package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotBackground;

@Autonomous(name="Trash Auto", group="Autonomous")
public class StrafeIntoPark extends LinearOpMode {

    RobotBackground robot = new RobotBackground();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        if (opModeIsActive())
        {
            robot.strafe(-40, 0.5);
        }
    }
}
