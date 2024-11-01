package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class AutoCommands {
    RobotBackground robot = new RobotBackground();

    public AutoCommands()
    {
        robot.arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.armMover1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMover1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.armMover2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMover2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void vertArmToPos(int pos)
    {
        robot.arm1.setTargetPosition(pos);
        robot.arm2.setTargetPosition(-pos);

        robot.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.arm1.setPower(0.8);
        robot.arm2.setPower(0.8);
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