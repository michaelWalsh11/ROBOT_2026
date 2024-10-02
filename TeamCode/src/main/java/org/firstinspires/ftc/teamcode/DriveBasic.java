package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="BIG FELLA", group="Training")
//@Disabled
public class DriveBasic extends OpMode {

    RobotBackground robot   = new RobotBackground(); // use the class created to define a Robot's hardware
    // could also use HardwarePushbotMatrix class.
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    private int armPos;
    private int dumbArmPos;
    private int armAnglerPos;
    private int dumbArmAnglerPos;

    private double ARM_SPEED = 0.7;
    private double ARM_SPEED_ANGLER = 0.7;

    private int rotateArm;
    private double topRungArm_Speed;

    private double DRIVE_SPEED = 0.5;

    private double CLIMB_POWER = 0.8;


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
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.armMover1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMover1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armMover2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMover2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotateArm = robot.armMover1.getCurrentPosition();
        rotateArm = robot.armMover2.getCurrentPosition();

        robot.arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        armPos = robot.arm1.getCurrentPosition();
        dumbArmPos = robot.arm2.getCurrentPosition();
        armAnglerPos = robot.armMover1.getCurrentPosition();
        dumbArmAnglerPos = robot.armMover2.getCurrentPosition();


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

    double previousLeftRearPower = 0.0;
    double previousLeftFrontPower = 0.0;
    double previousRightRearPower = 0.0;
    double previousRightFrontPower = 0.0;

    // Smoothing factor (value between 0 and 1)
    double smoothingFactor = 0.8; // make bigger to smooth smaller to accelerate

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        //claw open random guesses needs to be tuned
        if (gamepad1.right_bumper)
        {
            robot.rightHand.setPosition(1); // right hand
            robot.leftHand.setPosition(0);
        }
        if (gamepad1.left_bumper)
        {
            robot.rightHand.setPosition(0); // right hand
            robot.leftHand.setPosition(1);
        }


        //swivel (switch for now bc im lazy) test
        if (gamepad1.dpad_left)
        {
            robot.rotator.setPosition(1);
        }
        if (gamepad1.dpad_right)
        {
            robot.rotator.setPosition(0);
        }


        //drive
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

        if (!gamepad2.x) {
            robot.leftFront.setPower(leftFrontPower);
            robot.leftRear.setPower(leftRearPower);
            robot.rightFront.setPower(rightFrontPower);
            robot.rightRear.setPower(rightRearPower);
        }

        //ai generated move code to test:
        //mine is above


        //also try these later

//        leftX = gamepad1.left_stick_x;
//        leftY = gamepad1.left_stick_y;
//        rightX = gamepad1.right_stick_x ;
//        robot.driveFieldCentric(leftX, leftY, rightX, DRIVE_SPEED);
//        robot.driveStrafer(leftX, leftY, rightX);

//        double leftX = gamepad1.left_stick_x * DRIVE_SPEED;
//        double leftY = gamepad1.left_stick_y * DRIVE_SPEED;
//        double rightX = gamepad1.right_stick_x * DRIVE_SPEED;
//
//        // Calculate the target motor powers
//        double targetLeftRearPower = leftY + leftX - rightX;
//        double targetLeftFrontPower = leftY - leftX - rightX;
//        double targetRightRearPower = leftY - leftX + rightX;
//        double targetRightFrontPower = leftY + leftX + rightX;
//
//        // Smooth the motor power transitions
//        double leftRearPower = smoothingFactor * previousLeftRearPower + (1 - smoothingFactor) * targetLeftRearPower;
//        double leftFrontPower = smoothingFactor * previousLeftFrontPower + (1 - smoothingFactor) * targetLeftFrontPower;
//        double rightRearPower = smoothingFactor * previousRightRearPower + (1 - smoothingFactor) * targetRightRearPower;
//        double rightFrontPower = smoothingFactor * previousRightFrontPower + (1 - smoothingFactor) * targetRightFrontPower;
//
//        // Clip the power to ensure it's within the valid range
//        leftRearPower = Range.clip(leftRearPower, -1.0, 1.0);
//        leftFrontPower = Range.clip(leftFrontPower, -1.0, 1.0);
//        rightRearPower = Range.clip(rightRearPower, -1.0, 1.0);
//        rightFrontPower = Range.clip(rightFrontPower, -1.0, 1.0);
//
//        // Apply the smoothed power to the motors
//        robot.leftFront.setPower(leftFrontPower);
//        robot.leftRear.setPower(leftRearPower);
//        robot.rightFront.setPower(rightFrontPower);
//        robot.rightRear.setPower(rightRearPower);
//
//        // Store the current power values as previous values for the next loop
//        previousLeftRearPower = leftRearPower;
//        previousLeftFrontPower = leftFrontPower;
//        previousRightRearPower = rightRearPower;
//        previousRightFrontPower = rightFrontPower;
//
//        // Telemetry for motor powers
//        telemetry.addData("Left Front Power", leftFrontPower);
//        telemetry.addData("Left Rear Power", leftRearPower);
//        telemetry.addData("Right Front Power", rightFrontPower);
//        telemetry.addData("Right Rear Power", rightRearPower);






        //if we have a claw we can use this
//        if (gamepad1.right_bumper) {         // CLOSE
//            robot.leftHand.setPosition(0);
//            robot.rightHand.setPosition(1);
//        }
//        else if (gamepad1.left_bumper) {    // OPEN
//            robot.leftHand.setPosition(1);
//            robot.rightHand.setPosition(0);
//        }


        // Arm with DPADs for the funky JD arm
//        if (gamepad1.dpad_up)
//            robot.rightArm.setPower(0.5);
//        else if (gamepad1.dpad_down)
//            robot.rightArm.setPower(-0.5);
//        else
//            robot.rightArm.setPower(0.0);

        // Arm with DPADs for the funky JD arm
        double armAnglerMotorPower = gamepad2.right_trigger - gamepad2.left_trigger;
        if (armAnglerMotorPower > 0.4) {
            ARM_SPEED_ANGLER = 0.9;
            armAnglerPos += gamepad2.right_trigger * 20;
        }

        if (armAnglerMotorPower < -0.4) {
            ARM_SPEED_ANGLER = 0.9;
            armAnglerPos -= gamepad2.left_trigger * 20;
        }

        if (armAnglerPos > 0)
        {
            armAnglerPos = 0;
            ARM_SPEED_ANGLER = 0;
        }

        if (armAnglerPos < -1450)
        {
            armAnglerPos = -1450;
        }

        robot.armMover1.setPower(0.8);
        robot.armMover1.setTargetPosition(armAnglerPos);
        robot.armMover1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMover1.setPower(ARM_SPEED_ANGLER);

        robot.armMover2.setPower(0.8);
        robot.armMover2.setTargetPosition(-armAnglerPos);
        robot.armMover2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMover2.setPower(ARM_SPEED_ANGLER);


        //armMover
        double armMotorPower = gamepad1.right_trigger - gamepad1.left_trigger;
        if (armMotorPower > 0.4) {
            ARM_SPEED = 0.9;
            armPos += gamepad1.right_trigger * 60;
        }

        if (armMotorPower < -0.4) {
            ARM_SPEED = 0.9;
            armPos -= gamepad1.left_trigger * 60;
        }

        if (armPos > 4300)
        {
            armPos = 4300;
        }


        if (armPos < 0)
        {
            armPos = 0;
            ARM_SPEED = 0;
        }




        //armMover Action
        robot.arm1.setPower(armMotorPower); //try to delete this see what happens big fella
        robot.arm1.setTargetPosition(armPos);
        robot.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm1.setPower(ARM_SPEED);

        robot.arm2.setPower(armMotorPower);
        robot.arm2.setTargetPosition(-armPos);
        robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm2.setPower(ARM_SPEED);



        //show info slowbro (like the pokemon)
        telemetry.addLine("Wheels");

        telemetry.addData("Left Front Power", leftFrontPower);
        telemetry.addData("Left Rear Power", leftRearPower);
        telemetry.addData("Right Front Power", rightFrontPower);
        telemetry.addData("Right Rear Power", rightRearPower);

        telemetry.addLine("");
        telemetry.addLine("armMotorSpeeds");

        telemetry.addData("armSpeed", ARM_SPEED);
        telemetry.addData("SpeedArmAngler", ARM_SPEED_ANGLER);

        telemetry.addLine("");
        telemetry.addLine("armPos");

        telemetry.addLine("Arm " + armPos);
        telemetry.addData("Arm 2", dumbArmPos);

        telemetry.addLine("");
        telemetry.addLine("ArmAnglers");

        telemetry.addLine("ArmAngler " + armAnglerPos);
        telemetry.addData("ArmAngler 2", dumbArmAnglerPos);



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}


