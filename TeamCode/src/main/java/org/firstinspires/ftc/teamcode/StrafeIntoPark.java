package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Trash Auto", group="Autonomous")
public class StrafeIntoPark extends LinearOpMode {

    RobotBackground robot = new RobotBackground();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        if (opModeIsActive())
        {
            robot.strafe(48, 0.5);
        }
    }
}
