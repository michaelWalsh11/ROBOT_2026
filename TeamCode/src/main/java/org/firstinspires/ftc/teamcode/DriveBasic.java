package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="BIG FELLA", group="Training")
//@Disabled
public class DriveBasic extends OpMode {

    RobotBackground robot   = new RobotBackground(); // use the class created to define a Robot's hardware
    // could also use HardwarePushbotMatrix class.
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    private int armPos;
    private int OArmPos;
    private int armAnglerPos;
    private int OArmAnglerPos;

    private int number = 0;
    private boolean random = false;
    private boolean random2 = false;

    private double ARM_SPEED = 0.7;
    private double ARM_SPEED_ANGLER = 0.7;

    private int toggle = 0;

    private boolean rand1 = false;
    private boolean rand2 = false;

    private boolean bump1 = false;
    private boolean bump2 = false;

    private boolean other1 = false;
    private boolean other2 = false;

    private double servoPos = 0.0;
    private double spinPos = 0.0;
    private double handPos = 0.0;

    private int rotateArm;
    private double topRungArm_Speed;

    private double DRIVE_SPEED = 1.0;

    private double CLIMB_POWER = 0.8;


    @Override /* * */
    public void init() {

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


        spinPos = robot.spinner.getPosition();
        servoPos = robot.rotator.getPosition();


        armPos = robot.arm1.getCurrentPosition();
        OArmPos = robot.arm2.getCurrentPosition();
        armAnglerPos = robot.armMover1.getCurrentPosition();
        OArmAnglerPos = robot.armMover2.getCurrentPosition();


    }

    @Override /* * */
    public void init_loop()
    {

    }


    @Override /* * */
    public void start()
    {

    }

    @Override /* * */
    public void loop() {

        drive();
        grasper();
        arms();
        telemetry();
    }

    public void telemetry()
    {
        //show info slowbro (like the pokemon)
        telemetry.addLine("Wheels");

        telemetry.addLine("");
        telemetry.addLine("armMotorSpeeds");

        telemetry.addData("armSpeed", ARM_SPEED);
        telemetry.addData("SpeedArmAngler", ARM_SPEED_ANGLER);

        telemetry.addLine("");
        telemetry.addLine("armPos");

        telemetry.addLine("Arm " + armPos);
        telemetry.addData("Arm 2", OArmPos);

        telemetry.addLine("");
        telemetry.addLine("ArmAnglers");

        telemetry.addLine("ArmAngler " + armAnglerPos);
        telemetry.addData("ArmAngler 2", OArmAnglerPos);

        telemetry.addLine("");
        telemetry.addLine("Servos");
        telemetry.addLine("Graper1 " + robot.leftHand.getPosition());
        telemetry.addLine("Grasper2" + robot.rightHand.getPosition());
        telemetry.addLine("Spinner " + robot.spinner.getPosition());
        telemetry.addLine("Rotator " + robot.rotator.getPosition());

        //If daddy says
        //baby does
        //Never disobey daddy
        //OR ELSE !!!!!
        //(all said in a sigma alpha wolf tone)
        // by Miller Moore
    }

    public void armAnglerMover(int amp)
    {
        robot.armMover1.setPower(1.0);
        robot.armMover1.setTargetPosition(amp);
        robot.armMover1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMover1.setPower(ARM_SPEED_ANGLER);

        robot.armMover2.setPower(1.0);
        robot.armMover2.setTargetPosition(-amp);
        robot.armMover2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMover2.setPower(ARM_SPEED_ANGLER);
    }

    public void armMover(int amp, int number)
    {
            //armMover Action
        robot.arm1.setPower(1.0);
        robot.arm1.setTargetPosition(amp);
        robot.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm1.setPower(ARM_SPEED);

        robot.arm2.setPower(1.0);
        robot.arm2.setTargetPosition(-amp);
        robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm2.setPower(ARM_SPEED);
    }


    public void arms()
    {
        if (gamepad2.dpad_up)
        {
            ARM_SPEED = 1.0;
            armPos += 12;
        }
        if (gamepad2.dpad_down) {
            ARM_SPEED = 1.0;
            armPos -= 12;
        }

        if (armAnglerPos > 0)
        {
            armAnglerPos = 0;
        }

        if (gamepad1.x)
        {
            armAnglerPos = -140;
            robot.waitForTick(3000);
            armPos = 270;
            //armAnglerPos = -140;
        }

        if (armAnglerPos < -1450)
        {
            armAnglerPos = -1450;
        }

        if (gamepad2.b && !gamepad2.start)
        {
            armPos = 3700;
            armMover(armPos, number);
        }

        if (gamepad2.x)
        {
            armPos = 0;
            armMover(armPos, number);
        }

        if (gamepad2.y) {
            armAnglerPos = -1175;  // Move arm angler to preset position
            armAnglerMover(armAnglerPos); // Move arm angler
            toggle = 4300;
        }

        if (gamepad2.a && !gamepad2.start) {
            armAnglerPos = 0;  // Reset arm angler position
            armAnglerMover(armAnglerPos);
            toggle = 1650;
        }

        if (armAnglerPos < -1000)
        {
            toggle = 4300;
        }
        else {
            toggle = 1650;
        }

        armAnglerMover(armAnglerPos);


        //armMover
        double armMotorPower = gamepad2.left_stick_y;
        if (armMotorPower > 0.4) {
            ARM_SPEED_ANGLER = 1.0;
            armAnglerPos += (int) (gamepad2.left_stick_y * 15.0);
        }

        if (armMotorPower < -0.4) {
            ARM_SPEED_ANGLER = 1.0;
            armAnglerPos -= (int) (Math.abs(gamepad2.left_stick_y) * 15.0);
        }

        if (armPos > toggle)
        {

            armPos = toggle;
        }


        if (armPos < 0)
        {
            armPos = 0;
            ARM_SPEED = 0;
        }

        armMover(armPos, number);


    }

    public void grasper()
    {
        //open and close grasper
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

        //twist left and right
        if (gamepad2.left_bumper)
        {
            spinPos = Math.min(spinPos + 0.01, 1.0);;
        }
        if (gamepad2.right_bumper)
        {
            spinPos = Math.max(spinPos - 0.01, 0.0);
        }

        robot.spinner.setPosition(spinPos);

        //Probably
        double RSY = gamepad2.right_stick_y;
        if (RSY > 0.4)
        {
            servoPos = Math.min(servoPos + 0.01, 1.0);
        }
        if (RSY < -0.4)
        {
            servoPos = Math.max(servoPos - 0.01, 0.0);
        }

        robot.rotator.setPosition(servoPos);
    }

    public void drive()
    {
        //drive
        double leftX;
        double leftY;
        double rightX;

        if (DRIVE_SPEED == 1.0 && gamepad1.a && !rand1)
        {
            DRIVE_SPEED = 0.4;
            rand1 = true;
        }
        if (DRIVE_SPEED == 0.4 && gamepad1.a && !rand1)
        {
            DRIVE_SPEED = 1.0;
            rand1 = true;
        }

        if (!gamepad1.a)
        {
            rand1 = false;
        }

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

        //ai generated move code to test:
        //mine is above

        //also try these later

//        leftX = gamepad1.left_stick_x;
//        leftY = gamepad1.left_stick_y;
//        rightX = gamepad1.right_stick_x ;
//        //robot.driveFieldCentric(leftX, leftY, rightX, DRIVE_SPEED);
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
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {
        //todo
        //  N/A
    }

}