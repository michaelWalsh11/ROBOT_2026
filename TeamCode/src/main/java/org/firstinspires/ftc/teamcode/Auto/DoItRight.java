package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "We need good motion", group = "Autonomous")
public class DoItRight extends LinearOpMode
{

    //todo
    // - implement all the methods
    // - change the device names to be accurate
    // - add some hold code
    // - change set values to correct

    public class ArmA {

        private DcMotorEx armAngler1;
        private DcMotorEx armAngler2;

        public ArmA(HardwareMap hardwareMap) {
            armAngler1 = hardwareMap.get(DcMotorEx.class, "armMover1");
            armAngler1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armAngler1.setDirection(DcMotorSimple.Direction.REVERSE);

            armAngler2 = hardwareMap.get(DcMotorEx.class, "armMover2");
            armAngler2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armAngler2.setDirection(DcMotorSimple.Direction.FORWARD);

        }

        public class MoveArmAToPosition implements Action {
            private final double targetPosition;
            private final double holdingPower;
            private boolean initialized = false;

            public MoveArmAToPosition(double targetPosition, double holdingPower) {
                this.targetPosition = targetPosition;
                this.holdingPower = holdingPower;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized)
                {
                    armAngler1.setPower(0.8);
                    armAngler2.setPower(0.8);
                    initialized = true;
                }

                double pos = armAngler1.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < targetPosition) {
                    return true; // Continue running the action
                }
                else
                {
                    armAngler1.setPower(holdingPower);  // Apply a small power to hold the position
                    armAngler2.setPower(holdingPower);
                    return false; // Stop running, but maintain hold
                }
            }

            // Method to create actions with holding power
        }

        public Action armADown() {
            return new MoveArmAToPosition(0, 0.2);  // Add a holding power
        }

        public Action armAUp() {
            return new MoveArmAToPosition(1400, 0.2);   // Add a holding power
        }

    }


    //ArmV
    public class ArmV {

        private DcMotorEx arm1;
        private DcMotorEx arm2;

        public ArmV(HardwareMap hardwareMap) {
            arm1 = hardwareMap.get(DcMotorEx.class, "arm1");
            arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm1.setDirection(DcMotorSimple.Direction.FORWARD);

            arm2 = hardwareMap.get(DcMotorEx.class, "arm2");
            arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm2.setDirection(DcMotorSimple.Direction.REVERSE);


        }

        public class MoveArmToPosition implements Action {
            private final double targetPosition;
            private final double holdingPower;
            private boolean initialized = false;

            public MoveArmToPosition(double targetPosition, double holdingPower) {
                this.targetPosition = targetPosition;
                this.holdingPower = holdingPower;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized)
                {
                    arm1.setPower(0.8);
                    arm2.setPower(0.8);
                    initialized = true;
                }

                double pos = arm1.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < targetPosition) {
                    return true; // Continue running the action
                }
                else
                {
                    arm1.setPower(holdingPower);  // Apply a small power to hold the position
                    arm2.setPower(holdingPower);
                    return false; // Stop running, but maintain hold
                }
            }

        // Method to create actions with holding power
        }

        public Action armVSpecimenHigh() {
            return new MoveArmToPosition(1000, 0.2);  // Add a holding power
        }

        public Action armVSpecimenLow() {
            return new MoveArmToPosition(800, 0.2);   // Add a holding power
        }

        public Action armVHighBucket() {
            return new MoveArmToPosition(3700, 0.2);  // Add a holding power
        }

        public Action armVGround() {
            return new MoveArmToPosition(0, 0.2);     // Add a holding power
        }
    }

    //rotator
    public class Rotator
    {
        private Servo rotator;

        public Rotator(HardwareMap hardwareMap) {
            rotator = hardwareMap.get(Servo.class, "unknown");
        }

        public class Rotate1p0 implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(1.0);
                return false;
            }
        }

        public class Rotate0p2 implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(0.2);
                return false;
            }
        }

        public Action rotate1p0() {
            return new Rotate1p0();
        }

        public Action rotate0p2() {
            return new Rotate0p2();
        }
    }


    //angler
    public class Angler
    {
        private Servo angler;

        public Angler(HardwareMap hardwareMap) {
            angler = hardwareMap.get(Servo.class, "n");
        }

        public class AnglerStraight implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                angler.setPosition(0.5);
                return false;
            }
        }

        public class AnglerLeft implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                angler.setPosition(0.8);
                return false;
            }
        }

        public class AnglerRight implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                angler.setPosition(0.2);
                return false;
            }
        }

        public Action anglerRight() {
            return new AnglerRight();
        }

        public Action anglerLeft() {
            return new AnglerLeft();
        }

        public Action anglerStraight() {
            return new AnglerStraight();
        }
    }


    //grasper
    public class Grasp {
        private Servo leftGrasp;
        private Servo rightGrasp;

        public Grasp(HardwareMap hardwareMap) {
            leftGrasp = hardwareMap.get(Servo.class, "vic roy");
            rightGrasp = hardwareMap.get(Servo.class, "deuce");
        }

        public class OpenClaw implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightGrasp.setPosition(1.0);
                leftGrasp.setPosition(0.5);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }


        public class CloseClaw implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightGrasp.setPosition(0.5);
                leftGrasp.setPosition(1);
                return false;
            }
        }
        public Action closeClaw() {
            return new OpenClaw();
        }
    }



    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //gimble stuff
        Grasp grasper = new Grasp(hardwareMap);
        Rotator rotator = new Rotator(hardwareMap);
        Angler angler = new Angler(hardwareMap);

        ArmV armV = new ArmV(hardwareMap);
        ArmA armA = new ArmA(hardwareMap);


        TrajectoryActionBuilder driveToThing = drive.actionBuilder(initialPose)
                .lineToYConstantHeading(30)
                .waitSeconds(1);

        TrajectoryActionBuilder driveBack = drive.actionBuilder(initialPose)
                .lineToYConstantHeading(0);

        TrajectoryActionBuilder wait2 = drive.actionBuilder(initialPose)
                .waitSeconds(2);


        waitForStart();

        if (isStopRequested()) return;

        //very basic test
        Actions.runBlocking(
                new SequentialAction(
                        driveToThing.build(),
                        grasper.openClaw(),
                        driveBack.build()
                )
        );


        //should go forward, open claw, armV up, wait, armV down, open claw, go back also armA up and down in the middle
        Actions.runBlocking(
                new SequentialAction(
                        driveToThing.build(),
                        grasper.closeClaw(),
                        armA.armAUp(),
                        armV.armVSpecimenHigh(),
                        wait2.build(),
                        armA.armADown(),
                        armV.armVGround(),
                        grasper.openClaw(),
                        driveBack.build()
                )
        );

        while (opModeIsActive())
        {

        }
    }
}
