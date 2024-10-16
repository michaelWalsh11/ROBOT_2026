package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import javax.crypto.ExemptionMechanism;

@TeleOp(name = "Literronl an inch")
public class ImageFinder extends LinearOpMode {

    OpenCvCamera webcam;
    RobotBackground robot   = new RobotBackground();

    double servoPos = 0.0;
    int armPos = 0;
    double wristPos = 0.0;

    double centerWristPos = 0.725; //change to center wrist pos
    double scanningWristPos = 0.4; // change to a good angle to scan

    double rotatedAcceleration = 0.005;
    double wristAcceleration = 0.01;
    int armAcceleration = 10;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        // Initialize the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        ImageScannerPipeline scanner = new ImageScannerPipeline();
        // Set up a custom pipeline
        webcam.setPipeline(new ImageScannerPipeline());

        // Start streaming the camera feed
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera failed to open");
                telemetry.update();
            }
        });

        robot.arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armPos = robot.arm1.getCurrentPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start
        waitForStart();

        robot.rotator.setPosition(scanningWristPos);

        robot.rightHand.setPosition(0.65);
        robot.leftHand.setPosition(1);

        // While the OpMode is active, keep streaming and updating telemetry
        while (opModeIsActive()) {
            telemetry.update();

        }
    }

    class ImageScannerPipeline extends OpenCvPipeline {

        Mat hsvImage = new Mat();
        Mat redOnly = new Mat();
        Mat blueOnly = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            // Convert to HSV
            Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_BGR2HSV);

            // Red color detection
            Scalar lowRedLowerBound = new Scalar(0, 100, 100);
            Scalar lowRedUpperBound = new Scalar(10, 255, 255);
            Scalar highRedLowerBound = new Scalar(160, 100, 100);
            Scalar highRedUpperBound = new Scalar(179, 255, 255);

            Mat maskLowRed = new Mat();
            Mat maskHighRed = new Mat();
            Core.inRange(hsvImage, lowRedLowerBound, lowRedUpperBound, maskLowRed);
            Core.inRange(hsvImage, highRedLowerBound, highRedUpperBound, maskHighRed);
            Core.add(maskLowRed, maskHighRed, redOnly);

            // Blue color detection
            Scalar lowBlueLowerBound = new Scalar(100, 150, 50);
            Scalar lowBlueUpperBound = new Scalar(130, 255, 255);
            Core.inRange(hsvImage, lowBlueLowerBound, lowBlueUpperBound, blueOnly);

            // Clean up the masks
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
            Imgproc.morphologyEx(redOnly, redOnly, Imgproc.MORPH_CLOSE, kernel);
            Imgproc.morphologyEx(blueOnly, blueOnly, Imgproc.MORPH_CLOSE, kernel);

            // Find contours for red and blue areas
            List<MatOfPoint> redContours = new ArrayList<>();
            List<MatOfPoint> blueContours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(redOnly, redContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(blueOnly, blueContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Process contours (same logic as before)
            double minContourArea = 500.0;
            processContours(input, redContours, blueContours, minContourArea);

            return input;
        }

        private void processContours(Mat frame, List<MatOfPoint> redContours, List<MatOfPoint> blueContours, double minContourArea) {
            int wid = frame.width();
            int midX = wid / 2;

            int closestToMid = Integer.MAX_VALUE;
            int closestToMidY = Integer.MAX_VALUE;
            MatOfPoint closestContour = null;

            List<MatOfPoint> largeRedContours = new ArrayList<>();
            List<MatOfPoint> largeBlueContours = new ArrayList<>();

            for (MatOfPoint contour : redContours) {
                if (Imgproc.contourArea(contour) > minContourArea) {
                    Rect boundingRect = Imgproc.boundingRect(contour);
                    int contourCenterX = boundingRect.x + boundingRect.width / 2;
                    int contourCenterY = boundingRect.y + boundingRect.height / 2;
                    if (Math.abs(contourCenterX - midX) < Math.abs(closestToMid - midX)) {
                        closestToMid = contourCenterX;
                        closestToMidY = contourCenterY;
                        closestContour = contour;
                    }
                    largeRedContours.add(contour);
                }
            }

            for (MatOfPoint contour : blueContours) {
                if (Imgproc.contourArea(contour) > minContourArea) {
                    Rect boundingRect = Imgproc.boundingRect(contour);
                    int contourCenterX = boundingRect.x + boundingRect.width / 2;
                    int contourCenterY = boundingRect.y + boundingRect.height / 2;
                    if (Math.abs(contourCenterX - midX) < Math.abs(closestToMid - midX)) {
                        closestToMid = contourCenterX;
                        closestToMidY = contourCenterY;
                        closestContour = contour;
                    }
                    largeBlueContours.add(contour);
                }
            }

            if (closestContour != null) {
                // Calculate the minimum enclosing circle for the closest contour
                Point[] points = closestContour.toArray();
                RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(points));
                Rect closestBoundingRect = Imgproc.boundingRect(closestContour);
                Imgproc.rectangle(frame, new Point(closestBoundingRect.x, closestBoundingRect.y),
                        new Point(closestBoundingRect.x + closestBoundingRect.width, closestBoundingRect.y + closestBoundingRect.height),
                        new Scalar(0, 122, 122), 2);

                // Draw the rotated rectangle (orientation)
                Point[] rectPoints = new Point[4];
                rotatedRect.points(rectPoints);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(frame, rectPoints[i], rectPoints[(i + 1) % 4], new Scalar(255, 0, 0), 2);
                }

                // Calculate the orientation (angle)
                double angle = rotatedRect.angle - 90;
                if (rotatedRect.size.width < rotatedRect.size.height) {
                    angle += 90; // Adjust angle based on rectangle orientation
                }

                angle = (double) Math.round(angle * 100) / 100;

                if (angle > 15)
                {
                    servoPos += rotatedAcceleration;
                }
                else if (angle < -15)
                {
                    servoPos -= rotatedAcceleration;
                }
                else
                {
                    servoPos += 0.0;
                }

                robot.spinner.setPosition(servoPos);
            }

            for (MatOfPoint contour : largeRedContours) {
                if (contour != closestContour) {
                    Rect boundingRect = Imgproc.boundingRect(contour);
                    Imgproc.rectangle(frame, new Point(boundingRect.x, boundingRect.y),
                            new Point(boundingRect.x + boundingRect.width, boundingRect.y + boundingRect.height),
                            new Scalar(0, 0, 255), 2);  // Red contour box
                }
            }

            // Draw blue contours
            for (MatOfPoint contour : largeBlueContours) {
                if (contour != closestContour) {
                    Rect boundingRect = Imgproc.boundingRect(contour);
                    Imgproc.rectangle(frame, new Point(boundingRect.x, boundingRect.y),
                            new Point(boundingRect.x + boundingRect.width, boundingRect.y + boundingRect.height),
                            new Scalar(255, 0, 0), 2);  // Blue contour box
                }
            }
            double leftX;

            int mid = closestToMid - 320;
            if (mid > 15 && mid < 10000) {
                leftX = 0.1;
            }
            else if (mid < -15) {
                leftX = -0.1;
            }
            else {
                leftX = 0;
            }

            double leftRearPower = leftX;
            double leftFrontPower = -leftX;
            double rightRearPower = -leftX;
            double rightFrontPower = leftX;

            robot.leftFront.setPower(leftFrontPower);
            robot.leftRear.setPower(leftRearPower);
            robot.rightFront.setPower(rightFrontPower);
            robot.rightRear.setPower(rightRearPower);

            //todo
            // make it so that if the wristPos != to the pos where it is facing down then keep moving the arm forward
            int mid2 = closestToMidY - 240;
            if (mid2 > 15 && mid2 < 10000 || centerWristPos == wristPos)
            {
                armPos += armAcceleration;
            }
            else if (mid2 < -15)
            {
                wristPos += wristAcceleration;
            }
            else {
                armPos += 0;
                wristPos += 0;
            }

            if (armPos < 0)
            {
                armPos = 0;
            }

            robot.rotator.setPosition(wristPos);

            robot.armMover1.setTargetPosition(armPos);
            robot.armMover1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMover1.setPower(0.8);

            robot.armMover2.setTargetPosition(-armPos);
            robot.armMover2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMover2.setPower(0.8);

            // Use telemetry instead of print statements
            telemetry.addData("Closest Contour X ", mid);
            telemetry.addData("Closest Contour Y ", mid2);
            telemetry.addData("ArmPos ", armPos);
        }
    }
}