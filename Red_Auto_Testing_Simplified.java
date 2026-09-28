/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class Red_Auto_Testing_Simplified extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int Left = 0;
    int Middle = 1;
    int Right = 2;

    AprilTagDetection tagOfInterest = null;

    private DcMotor leftback;
    private DcMotor leftfront;
    private DcMotor rightback;
    private DcMotor rightfront;
    private CRServo slide;

    private CRServo slide1;
    private TouchSensor top;
    private TouchSensor mid;

    boolean condition_distance;
    boolean condition_angle;
    ElapsedTime PIDFTimer;
    float currentAngle;
    double error;
    int integral;
    double lastError;
    double deltaError;
    double derivative;
    PIDFCoefficients pidfGains;
    PIDFCoefficients pidfGains_angle;
    PIDFCoefficients pidfCoeffs;
    PIDFCoefficients pidfCoeffs_angle;

    int position;
    double drive_counts_per_inch;
    int hd_counts_per_rev;
    double drive_gear_reduction;
    double wheel_circumference;
    double drive_gear_per_mm;
    double arclength;

    @Override
    public void runOpMode() {
        telemetry.addLine("FUCK YOU");
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Spycam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        BNO055IMU.Parameters imuParameters;

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        leftback = hardwareMap.get(DcMotor.class, "leftback");
        leftfront = hardwareMap.get(DcMotor.class, "leftfront");
        rightback = hardwareMap.get(DcMotor.class, "rightback");
        rightfront = hardwareMap.get(DcMotor.class, "rightfront");
        slide = hardwareMap.get(CRServo.class, "slide");
        top = hardwareMap.get(TouchSensor.class, "top");
        mid = hardwareMap.get(TouchSensor.class, "mid");

        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
        claw.setPosition(0.58);
        integral = 0;
        lastError = 0;
        hd_counts_per_rev = 28;
        drive_gear_reduction = 15.12;
        currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftback.setDirection(DcMotorSimple.Direction.REVERSE);
        rightback.setDirection(DcMotorSimple.Direction.REVERSE);
        slide1.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        pidfCoeffs = new PIDFCoefficients(1e-10, 0.15, 2, 0);
        pidfGains = new PIDFCoefficients(0, 0, 0, 0);
        pidfCoeffs_angle = new PIDFCoefficients(0.5, 0, 0, 0);
        pidfGains_angle = new PIDFCoefficients(0, 0, 0, 0);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == Left || tag.id == Middle || tag.id == Right) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);

                    if (tagOfInterest.id == Left) {
                        //left code
                        telemetry.addLine("Go to left (Left code)");
                    } else if (tagOfInterest.id == Middle) {
                        //middle code
                        telemetry.addLine("Go to middle (Middle code)");
                    } else if (tagOfInterest.id == Right) {
                        //right code
                        telemetry.addLine("Go to right (Right code)");
                    } else{
                        telemetry.addLine("Null");
                    }
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        straightline(false,-0.7,50,0.0026);
        rotation(true,-0.7,45,35);
        straightline(false,-0.7,8.65,0.015);
        claw.setPosition(0.42);
        sleep(250);
        straightline(true,0.7,8.35,0.015);
        sleep(100);
        rotation(false,0.6,135,15);
        sleep(100);
        straightline(false,0.5,24.9,0.005);
        claw.setPosition(0.58);
        sleep(500);
        slide.setPower(-0.8);
        slide1.setPower(-0.8);
        sleep(1250);
        slide.setPower(0);
        slide1.setPower(0);
        straightline(true,-0.7,23.6,0.005);
        sleep(100);
        rotation(true,-0.7,135,15);
        straightline(false,-0.7,8.55,0.015);
        sleep(100);
        claw.setPosition(0.42);
        sleep(250);
        straightline(true,0.9,8.35,0.026);
        sleep(100);
        rotation(true,0.7,135,15);
        sleep(100);
        straightline(false,0.5,24.9,0.017);
        claw.setPosition(0.58);
        sleep(500);
        slide.setPower(-1);
        slide1.setPower(-1);
        sleep(1250);
        slide.setPower(0);
        slide1.setPower(-1);
        straightline(true,-0.7,23.6,0.005);
        sleep(100);
        rotation(true,-0.7,135,15);
        straightline(false,-0.7,8.55,0.015);
        sleep(100);
        claw.setPosition(0.42);
        sleep(250);
        rotation(false,0.7,135,15);

        /* Actually do something useful */
        if (tagOfInterest.id == Left) {
            straightline(false,0.5,25,0.017);

        } else if (tagOfInterest.id == Middle) {
            //middle code
            ((DcMotorEx) leftback).setVelocity(0);
            ((DcMotorEx) leftfront).setVelocity(0);
            ((DcMotorEx) rightback).setVelocity(0);
            ((DcMotorEx) rightfront).setVelocity(0);
            while (opModeIsActive() && !mid.isPressed()) {
                if (mid.isPressed()) {
                    slide.setPower(0);
                    slide1.setPower(0);
                }
            }

        } else if (tagOfInterest.id == Right) {
            straightline(true,0.5,25,0.017);

        } else{
            telemetry.addLine("You really fucked up");
        }
        telemetry.update();
        sleep(20);
    }
    public void straightline(boolean reverse, double slider, double distance, double speedratio){
        leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel_circumference = Math.PI * 98;
        drive_gear_per_mm = (hd_counts_per_rev * drive_gear_reduction) / wheel_circumference;
        drive_counts_per_inch = drive_gear_per_mm * 25.4;
        condition_angle = true;
        condition_distance = true;
        integral = 0;
        lastError = 0;
        position = (int) (drive_counts_per_inch * distance);
        //to set polarity of position
        if (reverse) {
            leftback.setTargetPosition(-position);
            leftfront.setTargetPosition(-position);
            rightback.setTargetPosition(-position);
            rightfront.setTargetPosition(-position);
        }
        else {
            leftback.setTargetPosition(position);
            leftfront.setTargetPosition(position);
            rightback.setTargetPosition(position);
            rightfront.setTargetPosition(position);
        }
        leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(slider);
        slide1.setPower(slider);
        while (opModeIsActive() && condition_distance){
            PIDF(position,speedratio);
            if (mid.isPressed() || top.isPressed()){
                slide.setPower(0);
                slide1.setPower(0);
            }
            }
        while (opModeIsActive() && (!mid.isPressed() || !top.isPressed())){
            slide.setPower(0);
            slide1.setPower(0);
        }
    }

    public void rotation(boolean clockwise, double slider, double degree, double speedratio_angle){
        arclength = Math.toRadians(degree)*26.632/25.4;
        leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel_circumference = Math.PI * 98;
        drive_gear_per_mm = (hd_counts_per_rev * drive_gear_reduction) / wheel_circumference;
        drive_counts_per_inch = drive_gear_per_mm * 25.4;
        condition_angle = true;
        condition_distance = true;
        integral = 0;
        lastError = 0;
        position = (int) (drive_counts_per_inch * arclength);
        //to set the direction of rotation
        if (clockwise){
            leftback.setTargetPosition(position);
            leftfront.setTargetPosition(position);
            rightback.setTargetPosition(-position);
            rightfront.setTargetPosition(-position);
        }
        else{
            leftback.setTargetPosition(-position);
            leftfront.setTargetPosition(-position);
            rightback.setTargetPosition(position);
            rightfront.setTargetPosition(position);
        }
        leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(slider);
        slide1.setPower(slider);
        while (opModeIsActive() && condition_distance){
            PIDF(position,speedratio_angle);
            if (mid.isPressed() || top.isPressed()){
                slide.setPower(0);
                slide1.setPower(0);
            }
        }
        while (opModeIsActive() && (!mid.isPressed() || !top.isPressed())){
            slide.setPower(0);
            slide1.setPower(0);
        }
    }
    public void PIDF(double targetPosition, double speedratio) {
        if (Math.abs(leftback.getCurrentPosition()) >= targetPosition - 2.5) {
            condition_distance = false;
        }
        int currentPosition;
        PIDFTimer.reset();
        currentPosition = leftback.getCurrentPosition();
        error = targetPosition - currentPosition;
        integral = (int) (integral + error * PIDFTimer.time());
        deltaError = error - lastError;
        derivative = deltaError / PIDFTimer.time();
        pidfGains.p = pidfCoeffs.p * error;
        pidfGains.i = pidfCoeffs.i * integral;
        pidfGains.d = pidfCoeffs.d * derivative;
        ((DcMotorEx) leftback).setVelocity(speedratio * ((pidfGains.p + pidfGains.i + pidfGains.d)));
        ((DcMotorEx) rightfront).setVelocity(speedratio * ((pidfGains.p + pidfGains.i + pidfGains.d)));
        ((DcMotorEx) leftfront).setVelocity(speedratio * ((pidfGains.p + pidfGains.i + pidfGains.d)));
        ((DcMotorEx) rightback).setVelocity(speedratio * ((pidfGains.p + pidfGains.i + pidfGains.d)) );
        lastError = error;
        if (Math.abs(leftback.getCurrentPosition()) >= targetPosition - 2) {
            condition_distance = false;
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));

    }
}