package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RRRB;
import org.firstinspires.ftc.teamcode.RobotBackground;

public class AutoCommands {
    RRRB robot = new RRRB();

    public AutoCommands(HardwareMap hardwareMap)
    {
        robot.init(hardwareMap);
    }

    public void graspAngler(double pos)
    {
        robot.spinner.setPosition(pos);
    }



    public class CloseGrasper implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // Check for null references
            if (robot.rightHand == null || robot.leftHand == null) {
                packet.put("Error", "Graspers not initialized");
                telemetry.update(); // Make sure telemetry can be updated
                return true; // End this action early
            }

            // Set positions for the graspers
            robot.rightHand.setPosition(1);
            robot.leftHand.setPosition(0.5);

            // Update telemetry with positions
            packet.put("rightHandPos", robot.rightHand.getPosition());
            packet.put("leftHandPos", robot.leftHand.getPosition());

            // It's crucial to update telemetry only if it's valid in this context
            try {
                telemetry.update();
            } catch (Exception e) {
                packet.put("TelemetryError", "Error updating telemetry: " + e.getMessage());
            }

            return true; // Return true to indicate this action has completed
        }
    }

    public Action closeClaw() {
        return new CloseGrasper();
    }

    public Action openGrasp()
    {
        robot.rightHand.setPosition(0.5);
        robot.leftHand.setPosition(1);

        return null;
    }

    public void openGrasper()
    {
        robot.rightHand.setPosition(0.5);
        robot.leftHand.setPosition(1);
    }

    public void closeGrasper()
    {
        robot.rightHand.setPosition(1);
        robot.leftHand.setPosition(0.5);
    }

    public void die()
    {
        robot.rightHand.setPosition(0.0);
    }

//    public void vertArmToPos(int pos)
//    {
//        robot.arm1.setTargetPosition(pos);
//        robot.arm2.setTargetPosition(-pos);
//
//        robot.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.arm1.setPower(0.8);
//        robot.arm2.setPower(0.8);
//    }

    public class VertArmToPos implements Action {
        private int targetPos;

        // Constructor to pass the target position
        public VertArmToPos(int pos) {
            this.targetPos = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robot.arm1.setTargetPosition(targetPos);
            robot.arm2.setTargetPosition(-targetPos);

            robot.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.arm1.setPower(0.8);
            robot.arm2.setPower(0.8);

            packet.put("Arm1 Target", targetPos);
            packet.put("Arm2 Target", -targetPos);
            packet.put("Arm1 Busy", robot.arm1.isBusy());
            packet.put("Arm2 Busy", robot.arm2.isBusy());
            telemetry.update();

            return false;  // Update as necessary based on desired behavior (e.g., return true when done)
        }
    }

    // Method to return the action
    public Action vertArmToPos(int pos) {
        return new VertArmToPos(pos);
    }





    public void vertArmToPosPower(int pos, double power)
    {
        robot.arm1.setTargetPosition(pos);
        robot.arm2.setTargetPosition(-pos);

        robot.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.arm1.setPower(power);
        robot.arm2.setPower(power);
    }

    public void horArmToPos(int pos)
    {
        robot.armMover1.setTargetPosition(-pos);
        robot.armMover2.setTargetPosition(pos);

        robot.armMover1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMover2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.armMover1.setPower(0.8);
        robot.armMover2.setPower(0.8);
    }

    public void horArmToPosPower(int pos, double power)
    {
        robot.armMover1.setTargetPosition(-pos);
        robot.armMover2.setTargetPosition(pos);

        robot.armMover1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMover2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.armMover1.setPower(power);
        robot.armMover2.setPower(power);
    }

    public void armsPos(int vPos, int hPos)
    {
        robot.armMover1.setTargetPosition(-hPos);
        robot.armMover2.setTargetPosition(hPos);

        robot.armMover1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMover2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.armMover1.setPower(0.8);
        robot.armMover2.setPower(0.8);


        robot.arm1.setTargetPosition(vPos);
        robot.arm2.setTargetPosition(-vPos);

        robot.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.arm1.setPower(0.8);
        robot.arm2.setPower(0.8);
    }

    public void waitCommand(int milliseconds)
    {
        try {
            Thread.sleep(milliseconds); // Pause execution for the specified time
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // Restore interrupted status
        }
    }

    public void armsPosPower(int vPos, int hPos, double power)
    {
        robot.armMover1.setTargetPosition(-hPos);
        robot.armMover2.setTargetPosition(hPos);

        robot.armMover1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMover2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.armMover1.setPower(power);
        robot.armMover2.setPower(power);


        robot.arm1.setTargetPosition(vPos);
        robot.arm2.setTargetPosition(-vPos);

        robot.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.arm1.setPower(power);
        robot.arm2.setPower(power);
    }

    public void resetEncoders() {
        robot.arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMover1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMover2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}