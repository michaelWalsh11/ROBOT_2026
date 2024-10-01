package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Line;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="RUN RUN NOW PLEASE RUN NOW", group="Training")
//@Disabled
public class LinearOpModeDriveNOWCopy extends OpMode {

    LinearOpModeBackgroundInfoCopy robot   = new LinearOpModeBackgroundInfoCopy(); // use the class created to define a Robot's hardware
    // could also use HardwarePushbotMatrix class.
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    private int armPos;
    private int armAnglerPos;
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

        robot.rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotateArm = robot.rightArm.getCurrentPosition();

        robot.leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armPos = robot.leftArm.getCurrentPosition();
        armAnglerPos = robot.rightArm.getCurrentPosition();


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



        //drive
//        double leftX;
//        double leftY;
//        double rightX;
//
//        leftX = gamepad1.left_stick_x * DRIVE_SPEED;
//        leftY = gamepad1.left_stick_y * DRIVE_SPEED;
//        rightX = gamepad1.right_stick_x * DRIVE_SPEED;
//
//        double leftRearPower = leftY + leftX - rightX;
//        double leftFrontPower = leftY - leftX - rightX;
//        double rightRearPower = leftY - leftX + rightX;
//        double rightFrontPower = leftY + leftX + rightX;
//
//        if (!gamepad1.x)
//        {
//            robot.leftFront.setPower(leftFrontPower);
//            robot.leftRear.setPower(leftRearPower);
//            robot.rightFront.setPower(rightFrontPower);
//            robot.rightRear.setPower(rightRearPower);
//        }
//        else {
//            robot.leftFront.setPower(leftFrontPower * 4);
//            robot.leftRear.setPower(leftRearPower * 4);
//            robot.rightFront.setPower(rightFrontPower * 4);
//            robot.rightRear.setPower(rightRearPower * 4);
//        }

        //ai generated move code to test:
        //mine is above


        //also try these later

//        leftX = gamepad1.left_stick_x;
//        leftY = gamepad1.left_stick_y;
//        rightX = gamepad1.right_stick_x ;
//        robot.driveFieldCentric(leftX, leftY, rightX, DRIVE_SPEED);
//        robot.driveStrafer(leftX, leftY, rightX);

        double leftX = gamepad1.left_stick_x * DRIVE_SPEED;
        double leftY = gamepad1.left_stick_y * DRIVE_SPEED;
        double rightX = gamepad1.right_stick_x * DRIVE_SPEED;

        // Calculate the target motor powers
        double targetLeftRearPower = leftY + leftX - rightX;
        double targetLeftFrontPower = leftY - leftX - rightX;
        double targetRightRearPower = leftY - leftX + rightX;
        double targetRightFrontPower = leftY + leftX + rightX;

        // Smooth the motor power transitions
        double leftRearPower = smoothingFactor * previousLeftRearPower + (1 - smoothingFactor) * targetLeftRearPower;
        double leftFrontPower = smoothingFactor * previousLeftFrontPower + (1 - smoothingFactor) * targetLeftFrontPower;
        double rightRearPower = smoothingFactor * previousRightRearPower + (1 - smoothingFactor) * targetRightRearPower;
        double rightFrontPower = smoothingFactor * previousRightFrontPower + (1 - smoothingFactor) * targetRightFrontPower;

        // Clip the power to ensure it's within the valid range
        leftRearPower = Range.clip(leftRearPower, -1.0, 1.0);
        leftFrontPower = Range.clip(leftFrontPower, -1.0, 1.0);
        rightRearPower = Range.clip(rightRearPower, -1.0, 1.0);
        rightFrontPower = Range.clip(rightFrontPower, -1.0, 1.0);

        // Apply the smoothed power to the motors
        robot.leftFront.setPower(leftFrontPower);
        robot.leftRear.setPower(leftRearPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightRear.setPower(rightRearPower);

        // Store the current power values as previous values for the next loop
        previousLeftRearPower = leftRearPower;
        previousLeftFrontPower = leftFrontPower;
        previousRightRearPower = rightRearPower;
        previousRightFrontPower = rightFrontPower;

        // Telemetry for motor powers
        telemetry.addData("Left Front Power", leftFrontPower);
        telemetry.addData("Left Rear Power", leftRearPower);
        telemetry.addData("Right Front Power", rightFrontPower);
        telemetry.addData("Right Rear Power", rightRearPower);






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
            ARM_SPEED_ANGLER = 1.0;
            armAnglerPos += gamepad2.right_trigger * 10;
        }

        if (armAnglerMotorPower < -0.4) {
            ARM_SPEED = 1.0;
            armAnglerPos -= gamepad2.left_trigger * 10;
        }

        if (armAnglerPos > 1320) {
            armAnglerPos = 1319;
        }

        if (armAnglerPos < 0) {
            armAnglerPos = 1;
        }

        if (gamepad2.a)
        {
            armAnglerPos = 1;
        }
        if (gamepad2.y)
        {
            armAnglerPos = 1319;
        }

        robot.rightArm.setPower(0.8);
        robot.rightArm.setTargetPosition(armAnglerPos);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightArm.setPower(ARM_SPEED_ANGLER);



        //armMover
        double armMotorPower = gamepad1.right_trigger - gamepad1.left_trigger;
        if (armMotorPower > 0.4) {
            ARM_SPEED = 1.0;
            armPos += gamepad1.right_trigger * 10;
        }

        if (armMotorPower < -0.4) {
            ARM_SPEED = 1.0;
            armPos -= gamepad1.left_trigger * 10;
        }

        //safety code
        if (armPos > 4350) {
            armPos = 4349;
        }

        if (armPos < 0) {
            armPos = 1;
        }

        if (gamepad1.a)
        {
            armPos = 1;
        }
        if (gamepad1.y)
        {
            armPos = 4349;
        }

        //extraneous commands

        //put it in
        if (gamepad1.dpad_down)
        {
            armAnglerPos = 200;
            armPos = 1200;

            try {
                robot.wait(1);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            armAnglerPos = 0;
        }




        //armMover Action
        robot.leftArm.setPower(armMotorPower);
        robot.leftArm.setTargetPosition(armPos);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftArm.setPower(ARM_SPEED);

        telemetry.addData("ArmAnglerPos", armAnglerPos);
        telemetry.addData("ArmPos", armPos);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}


