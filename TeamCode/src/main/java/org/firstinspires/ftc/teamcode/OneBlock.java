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

            robot.move(5, 0.8);
            robot.armsMoveToVert(0.8, -1000);
            robot.armsMoveToPos(0.8, 1000);

//            robot.closeGrasper();

            robot.waitForTick(3000); // 3 seconds?

            robot.move(2, 0.8);

            robot.rotator.setPosition(0.75);

            robot.waitForTick(3000);

            robot.armsMoveToPos(0.8, -300);
            robot.closeGrasper();

            robot.waitForTick(3000);
//
//            robot.waitForTick(1000);
//
//            robot.move(-10, 0.8);
//            robot.armsMoveToVert(0.8, 1000);
//            robot.armsMoveToPos(0.8, -1700);
//
//            robot.waitForTick(3000);
//
//            robot.strafe(40, 0.8);
        }
    }
}