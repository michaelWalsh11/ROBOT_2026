package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Hold Into top rung", group="Autonomous")
public class OneBlock extends LinearOpMode {

    RobotBackground robot = new RobotBackground();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        if (opModeIsActive())
        {
            robot.closeGrasper();

            robot.move(50, 0.8);
            robot.armsMoveToVert(0.8, 700);
            robot.armsMoveToPos(0.8, 2500);

            robot.waitForTick(3000); // 3 seconds?

            robot.armsMoveToPos(0.8, 2100);
            robot.openGrasper();

            robot.waitForTick(1000);

            robot.move(-50, 0.8);
            robot.armsMoveToVert(0.8, 0);
            robot.armsMoveToPos(0.8, 0);

            robot.waitForTick(3000);

            robot.strafe(48, 0.8);
        }
    }
}