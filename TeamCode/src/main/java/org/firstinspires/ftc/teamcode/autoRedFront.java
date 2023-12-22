package org.firstinspires.ftc.teamcode;

/*
 * Copyright (c) 2019 OpenFTC Team
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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.google.android.gms.tasks.OnFailureListener;
//import com.google.android.gms.tasks.OnSuccessListener;
//import com.google.android.gms.tasks.Task;
//import com.google.mlkit.vision.barcode.BarcodeScanner;
//import com.google.mlkit.vision.barcode.BarcodeScannerOptions;
//import com.google.mlkit.vision.barcode.BarcodeScanning;
//import com.google.mlkit.vision.barcode.common.Barcode;
//import com.google.mlkit.vision.common.InputImage;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.lang.reflect.Array;
import java.util.List;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous
public class autoRedFront extends LinearOpMode
{
    /** [START] CLASS VARIABLES GO HERE. THEY CAN BE USED IN ANY FUNCTION BELOW THIS POINT. **/
    final double POT_HIGH = 3.0;
    final double POT_BACKBOARD = 2.0;
    final double POT_GROUND = 1.1;
    private enum parkPlace { PARKNEAR, PARKMIDDLE, PARKFAR};
    private enum goPlace { GOLEFT, GOCENTER, GORIGHT };

    private parkPlace parkingSpot = parkPlace.PARKFAR;

//    private String rawValue = "-1";
//    String model;
//    boolean USE_WEBCAM;
//    TfodProcessor myTfodProcessor;
//    int middleOfScreenX;
//    VisionPortal myVisionPortal;
    // [START] webcam barcode scanner variables
//    private OpenCvWebcam webcam;
    // [END] webcam barcode scanner variables

    //    private DcMotor lift;
//    private TouchSensor magLo;
//    private CRServo intake;
    private Servo rightSpike;
    private Servo leftSpike;
    private Servo rightPixelGate;
    private Servo leftPixelGate;
    private DistanceSensor leftDistance;
    private DistanceSensor rightDistance;
    private DcMotor arm1;
    private DcMotor arm2;
    private AnalogInput pot1;
    private Servo extender;
    /** [END] CLASS VARIABLES GO HERE. THEY CAN BE USED IN ANY FUNCTION BELOW THIS POINT. **/

    private void raiseArm(boolean high)
    {
        double position = high? POT_HIGH: POT_BACKBOARD;
        while ((pot1.getVoltage() < position) && opModeIsActive())
        {
            setArmPowers(0.5);
        }
        setArmPowers(0);
    }

    private void lowerArm(boolean ground)
    {
        double position = ground? POT_GROUND: POT_BACKBOARD;

        while ((pot1.getVoltage() > position) && opModeIsActive())
        {
            setArmPowers(-0.03);
        }
        setArmPowers(0);
    }
    private goPlace choiceFunction(boolean leftAligned)
    {
        goPlace ret = goPlace.GOCENTER;

        int leftReading = 350;
        int rightReading = 350;
        int minLeftDistance = 350;
        int minRightDistance = 350;


        for (int count = 0; count < 10; count++)
        {
            leftReading = (int) leftDistance.getDistance(DistanceUnit.INCH);
            rightReading = (int) rightDistance.getDistance(DistanceUnit.INCH);
            if (minLeftDistance > leftReading)
            {
                minLeftDistance = leftReading;
            }
            if (minRightDistance > rightReading)
            {
                minRightDistance = rightReading;
            }
        }

        if (leftAligned)
        {
            if (minLeftDistance < 20)
            {
                ret = goPlace.GOLEFT;
            }
            else if (minRightDistance < 20)
            {
                ret = goPlace.GOCENTER;
            }
            else
            {
                ret = goPlace.GORIGHT;
            }
        }
        else
        {
            if (minLeftDistance < 20)
            {
                ret = goPlace.GOCENTER;
            }
            else if (minRightDistance < 20)
            {
                ret = goPlace.GORIGHT;
            }
            else
            {
                ret = goPlace.GOLEFT;
            }
        }

        return ret;
    }

    private void setArmPowers(double power)
    {
        arm1.setPower(power);
        arm2.setPower(power);
    }

    private void choosePark()
    {
        if (gamepad1.dpad_left)
        {
            parkingSpot = parkPlace.PARKNEAR;
        }
        else if (gamepad1.dpad_up || gamepad1.dpad_down)
        {
            parkingSpot = parkPlace.PARKMIDDLE;
        }
        else if (gamepad1.dpad_right)
        {
            parkingSpot = parkPlace.PARKFAR;
        }
        telemetry.addData("Parking:", ((parkPlace.PARKFAR==parkingSpot)?"FAR":((parkPlace.PARKNEAR==parkingSpot)?"NEAR":"MIDDLE")));
    }

    private void setSpike(boolean left, boolean up)
    {
        if (left)
        {
            if (up)
            {
                // up
                leftSpike.setPosition(1);
            }
            else
            {
                // down
                leftSpike.setPosition(0);
            }
        }
        else // right
        {
            if (up)
            {
                // up
                rightSpike.setPosition(0);
            }
            else
            {
                // down
                rightSpike.setPosition(1);
            }
        }
    }

    private void setGate(boolean left, boolean open)
    {
        if (left)
        {
            if (open)
            {
                // open
                leftPixelGate.setPosition(1);
            }
            else
            {
                // closed
                leftPixelGate.setPosition(0);
            }
        }
        else // right
        {
            if (open)
            {
                // open
                rightPixelGate.setPosition(0);
            }
            else
            {
                // closed
                rightPixelGate.setPosition(1);
            }
        }
    }

    private void dropPixelOnBackboard()
    {
//      extender.setPosition(0);

//      sleep(1000);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lowerArm(false);
        sleep(500);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setGate(true, true);
        setGate(false, true);
    }

    // This code moves the robot to left position.
    /** This parks in Station 1 **/

    /** [END] CLASS VARIABLES GO HERE. THEY CAN BE USED IN ANY FUNCTION BELOW THIS POINT. **/
    private Pose2d goRight(SampleMecanumDrive drive, Pose2d startPose)
    {
//        drive.setPoseEstimate(startPose);

        // go to spike mark
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-54,-30, Math.toRadians(90)))
                .waitSeconds(0.5)
                .build();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq1.end())
                .waitSeconds(0.4)
                .lineToLinearHeading(new Pose2d(-54,-61, Math.toRadians(90)))
                .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(36,-62, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(48,-43, Math.toRadians(0)))
                .forward(2)
                .waitSeconds(1)
                .build();


        drive.followTrajectorySequence(trajSeq1);

        setSpike(false, true);

        drive.followTrajectorySequence(trajSeq2);

        dropPixelOnBackboard();

        return trajSeq2.end();
    }

    private Pose2d goCenter(SampleMecanumDrive drive, Pose2d startPose)
    {
//        drive.setPoseEstimate(startPose);

        // go to spike mark
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-43,-24, Math.toRadians(90)))
                .waitSeconds(0.5)
                .build();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq1.end())
                .waitSeconds(0.4)
                .lineToLinearHeading(new Pose2d(-40,-61, Math.toRadians(90)))
                .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(36,-62, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(48,-37, Math.toRadians(0)))
                .forward(2)
                .waitSeconds(1)
                .build();


        drive.followTrajectorySequence(trajSeq1);

        setSpike(false, true);

        drive.followTrajectorySequence(trajSeq2);

        dropPixelOnBackboard();

        return trajSeq2.end();
    }

    private Pose2d goLeft(SampleMecanumDrive drive, Pose2d startPose)
    {
//        drive.setPoseEstimate(startPose);

        // go to spike mark
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-32.25,-30, Math.toRadians(90)))
                .waitSeconds(0.5)
                .build();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq1.end())
                .waitSeconds(0.4)
                .lineToLinearHeading(new Pose2d(-40,-61, Math.toRadians(90)))
                .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(36,-62, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(48,-31, Math.toRadians(0)))
                .forward(2)
                .waitSeconds(1)
                .build();


        drive.followTrajectorySequence(trajSeq1);

        setSpike(false, true);

        drive.followTrajectorySequence(trajSeq2);

        dropPixelOnBackboard();

        return trajSeq2.end();
    }

    private void parkNear(SampleMecanumDrive drive, Pose2d startPose)
    {
//        drive.setPoseEstimate(startPose);

        // go to spike mark
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(48,-48, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(48,-58, Math.toRadians(180)))
                .build();

//        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq1.end())
//                .waitSeconds(0.4)
////                .strafeLeft(9)
//                .lineToLinearHeading(new Pose2d(54,27, Math.toRadians(0)))
//                .forward(2)
//                .waitSeconds(1)
////                .lineTo(new Vector2d(52, 12))
//                .build();


        drive.followTrajectorySequence(trajSeq1);

        setSpike(true, true);

        sleep(500);
    }

    private void parkMiddle(SampleMecanumDrive drive, Pose2d startPose)
    {
////        drive.setPoseEstimate(startPose);
//
//        // go to spike mark
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(48,-34.5, Math.toRadians(0)))
                .turn(Math.toRadians(180))
                .build();
//
//        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq1.end())
//                .waitSeconds(0.4)
//                .lineToLinearHeading(new Pose2d(54,34.5, Math.toRadians(0)))
////                .strafeRight(9)
//                .forward(2)
//                .waitSeconds(1)
////                .lineTo(new Vector2d(52, 12))
//                .build();
//
//
        drive.followTrajectorySequence(trajSeq1);

        setSpike(true, true);

        sleep(500);
    }

    private void parkFar(SampleMecanumDrive drive, Pose2d startPose)
    {
//        drive.setPoseEstimate(startPose);

        // go to spike mark
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(48,-24, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(48,-12, Math.toRadians(180)))
                .build();

//        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq1.end())
//                .waitSeconds(0.4)
//                .lineToLinearHeading(new Pose2d(54,41, Math.toRadians(0)))
//                .forward(2)
//                .waitSeconds(1)
////                .lineTo(new Vector2d(52, 12))
//                .build();


        drive.followTrajectorySequence(trajSeq1);

        setSpike(true, true);

        sleep(500);
    }

    @Override
    public void runOpMode()
    {
        leftDistance = hardwareMap.get(DistanceSensor.class, "leftDistance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "rightDistance");

        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");

        leftSpike = hardwareMap.get(Servo.class, "leftSpike");
        rightSpike = hardwareMap.get(Servo.class, "rightSpike");

        leftPixelGate = hardwareMap.get(Servo.class, "leftPixel");
        rightPixelGate = hardwareMap.get(Servo.class, "rightPixel");

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pot1 = hardwareMap.get(AnalogInput.class, "pot1");

        extender = hardwareMap.get(Servo.class, "extender");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-40, -64, Math.toRadians(90));
        Pose2d endPose;

        while (!opModeIsActive() && !isStopRequested())
        {
            choosePark();
//            telemetryTfod();
            // Push telemetry to the Driver Station.
            telemetry.addData("leftDistance", "%3.0f", leftDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("rightDistance", "%3.0f", rightDistance.getDistance(DistanceUnit.INCH));
//            telemetry.addData("Going:", "CENTER");
            telemetry.update();
        }

        /** NOTE: WE ARE MOVING DURING INIT TO RAISE THE LIFT SO THE CAMERA CAN SEE THE BARCODE! **/
//        intake.setPower(-1);
        /** THIS MEANS WE MUST HAVE THE "ROBOT MOVES DURING INITIALIZATION" STICKER ON OUR ROBOT! **/

        /** Wait for the user to press the "PLAY" button on the Driver Station **/

        waitForStart();
        /** [END] THIS IS OUR INIT CODE. IT RUNS WHEN THE "INIT" BUTTON IS PRESSED. **/

        /** [START] THIS IS OUR AUTONOMOUS CODE. IT RUNS WHEN THE "PLAY" BUTTON IS PRESSED. **/
        while (opModeIsActive())
        {
            /** This stops the webcam barcode scanner since we don't need it any more **/
//            stopWebcam();
            setSpike(true,false);
            setSpike(false,false);

            sleep(500);

            /* This is our main AUTO code... */
            drive.setPoseEstimate(startPose);
            TrajectorySequence trajSeqStart = drive.trajectorySequenceBuilder(startPose)
                    .forward(17)
                    .waitSeconds(0.5)
                    .build();

            drive.followTrajectorySequence(trajSeqStart);
            // This figures out what parking space to go to.
            goPlace placeValue = choiceFunction(true);
            switch (placeValue)
            {
                case GOLEFT:
                {
                    telemetry.addData("Going:", "LEFT");
                    telemetry.update();
                    endPose = goLeft(drive, trajSeqStart.end());
                    break;
                }
                case GORIGHT:
                {
                    telemetry.addData("Going:", "RIGHT");
                    telemetry.update();
                    endPose = goRight(drive, trajSeqStart.end());
                    break;
                }
                case GOCENTER:
                default:
                {
                    telemetry.addData("Going:", "CENTER");
                    telemetry.update();
                    endPose = goCenter(drive, trajSeqStart.end());
                    break;
                }
            }

            setArmPowers(0.15);
            setGate(true,false);
            setGate(false,false);

            switch (parkingSpot)
            {
                case PARKNEAR:
                {
                    parkNear(drive, endPose);
                    break;
                }
                case PARKFAR:
                {
                    parkFar(drive, endPose);
                    break;
                }
                case PARKMIDDLE:
                default:
                {
                    parkMiddle(drive, endPose);
                    break;
                }
            }

            setArmPowers(-0.03);
            /*
             * This is the end of our op mode so just wait
             * here forever (ONLY while the op mode is active)
             * otherwise we would go back to the top and start over
             */
            while (opModeIsActive())
            {
                sleep(100);
            }
        }
        /** [END] THIS IS OUR AUTONOMOUS CODE. IT RUNS WHEN THE "PLAY" BUTTON IS PRESSED. **/
    } // END runOpMode()
//This code moves the robot to left postion.
    /** This parks in Station 1 **/


    /**********************************************************************************************/
    /*********************** WEBCAM BARCODE SCANNING CODE IS BELOW THIS LINE **********************/
    /**********************************************************************************************/
//    private void startWebcam()
//    {
//        /*
//         * Instantiate an OpenCvCamera object for the camera we'll be using.
//         * In this sample, we're using a webcam. Note that you will need to
//         * make sure you have added the webcam to your configuration file and
//         * adjusted the name here to match what you named it in said config file.
//         */
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        // Do Not Activate the Camera Monitor View
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
//
//        /*
//         * Specify the image processing pipeline we wish to invoke upon receipt
//         * of a frame from the camera. Note that switching pipelines on-the-fly
//         * (while a streaming session is in flight) *IS* supported.
//         */
//        webcam.setPipeline(new SamplePipeline());
//
//        /*
//         * Open the connection to the camera device. New in v1.4.0 is the ability
//         * to open the camera asynchronously, and this is now the recommended way
//         * to do it. The benefits of opening async include faster init time, and
//         * better behavior when pressing stop during init (i.e. less of a chance
//         * of tripping the stuck watchdog)
//         *
//         * If you really want to open synchronously, the old method is still available.
//         */
//        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                /*
//                 * Tell the webcam to start streaming images to us! Note that you must make sure
//                 * the resolution you specify is supported by the camera. If it is not, an exception
//                 * will be thrown.
//                 *
//                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
//                 * supports streaming from the webcam in the uncompressed YUV image format. This means
//                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
//                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
//                 *
//                 * Also, we specify the rotation that the webcam is used in. This is so that the image
//                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
//                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
//                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
//                 * away from the user.
//                 */
//                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//                telemetry.addLine("COULDN'T OPEN WEBCAM!!!");
//            }
//        });
//    }
//
//    private void stopWebcam()
//    {
//        webcam.stopStreaming();
//        webcam.closeCameraDevice();
//        sleep(100);
//    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
//    class SamplePipeline extends OpenCvPipeline
//    {
//        boolean viewportPaused;
//
//        /*
//         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
//         * highly recommended to declare them here as instance variables and re-use them for
//         * each invocation of processFrame(), rather than declaring them as new local variables
//         * each time through processFrame(). This removes the danger of causing a memory leak
//         * by forgetting to call mat.release(), and it also reduces memory pressure by not
//         * constantly allocating and freeing large chunks of memory.
//         */
//
//        @Override
//        public Mat processFrame(Mat input)
//        {
//            sleep(1000);
//            /*
//             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
//             * will only dereference to the same image for the duration of this particular
//             * invocation of this method. That is, if for some reason you'd like to save a copy
//             * of this particular frame for later use, you will need to either clone it or copy
//             * it to another Mat.
//             */
//            if (input.isContinuous())
//            {
//                Bitmap bmp = Bitmap.createBitmap(640, 480, Bitmap.Config.ARGB_8888);
//                Utils.matToBitmap(input, bmp);
//                image = InputImage.fromBitmap(bmp, 0);
//
////                    File file = new File("/storage/emulated/0/FIRST/matchlogs/webcam_000.bmp");
////                    try (FileOutputStream fos = new FileOutputStream(file))
////                    {
////                        ByteArrayOutputStream stream = new ByteArrayOutputStream();
////                        bmp.compress(Bitmap.CompressFormat.PNG, 100, stream);
////
////                        fos.write(stream.toByteArray());
////                        Log.i("BARCODE:","Successfully written data to the file");
////                    } catch (IOException e) {
////                        e.printStackTrace();
////                    }
//
//                Task<List<Barcode>> result = scanner.process(image).addOnSuccessListener(new OnSuccessListener<List<Barcode>>()
//                        {
//                            @Override
//                            public void onSuccess(List<Barcode> barcodes)
//                            {
//                                // Task completed successfully
//                                for (Barcode barcode : barcodes)
//                                {
//                                    Rect bounds = barcode.getBoundingBox();
//                                    android.graphics.Point[] corners = barcode.getCornerPoints();
//
//                                    rawValue = barcode.getRawValue();
//                                    Log.w("BARCODE:", "[" + rawValue + "]");
//                                    telemetry.addData("BARCODE:", "[" + rawValue + "]");
//                                    /*
//                                     * Send some stats to the telemetry
//                                     */
//                                    telemetry.addData("Frame Count", webcam.getFrameCount());
//                                    telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
//                                    telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
////                                telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
////                                telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
////                                telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
//                                    telemetry.update();
//                                }
//                            }
//                        })
//                        .addOnFailureListener(new OnFailureListener()
//                        {
//                            @Override
//                            public void onFailure(@NonNull Exception e)
//                            {
//                                // Task failed with an exception
//                                Log.e("BARCODE:", e.toString());
//                                telemetry.addData("BARCODE:", e.toString());
//                                telemetry.update();
//                            }
//                        });
//            }
//
//            /*
//             * Draw a simple box around the middle 1/2 of the entire frame
//             */
//            Imgproc.rectangle(
//                    input,
//                    new Point(
//                            input.cols() / 4,
//                            input.rows() / 4),
//                    new Point(
//                            input.cols() * (3f / 4f),
//                            input.rows() * (3f / 4f)),
//                    new Scalar(0, 255, 0), 4);
//
//            /**
//             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
//             * to change which stage of the pipeline is rendered to the viewport when it is
//             * tapped, please see {@link PipelineStageSwitchingExample}
//             */
//            return input;
//        }
//
//        @Override
//        public void onViewportTapped()
//        {
//            /*
//             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
//             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
//             * when you need your vision pipeline running, but do not require a live preview on the
//             * robot controller screen. For instance, this could be useful if you wish to see the live
//             * camera preview as you are initializing your robot, but you no longer require the live
//             * preview after you have finished your initialization process; pausing the viewport does
//             * not stop running your pipeline.
//             *
//             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
//             */
//
//            viewportPaused = !viewportPaused;
//
//            if (viewportPaused)
//            {
//                webcam.pauseViewport();
//            }
//            else
//            {
//                webcam.resumeViewport();
//            }
//        }
//    }

} // END class centerstage
