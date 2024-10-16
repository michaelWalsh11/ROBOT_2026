package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="HUNTER RUN THIS RN", group="Training")
//@Disabled
public class ServoTester extends OpMode {

    RobotBackground robot   = new RobotBackground(); // use the class created to define a Robot's hardware
    // could also use HardwarePushbotMatrix class.
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Robot Ready");    //

        // Set to Run without Encoder for Tele Operated


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {


        //armPos = 50; // Lifts Arm for Driving
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        //todo
        // - hotkeys
        // - try to fix the arm going down being ass movement
        // - battery holder
        // - fix the grasper
        // - fix dangling wires


        //claw open random guesses needs to be tuned
        double x = robot.rightHand.getPosition();

        if (gamepad2.right_trigger > 0.4)
        {
            robot.rightHand.setPosition(0.5);
            robot.leftHand.setPosition(1);
        }
        if (gamepad2.left_trigger > 0.4)
        {
            robot.rightHand.setPosition(1);
            robot.leftHand.setPosition(0.5);
        }

        if (gamepad2.dpad_up)
        {
            robot.rightHand.setPosition(x += 0.009);
        }
        if (gamepad2.dpad_down)
        {
            robot.rightHand.setPosition(x -= 0.009);
        }
        telemetry.addLine("Hello Hunter!" + x);


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
