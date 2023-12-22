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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous
public class autoRedFrontFarPark extends LinearOpMode
{
    private int choiceFunction(
            // TODO: Enter the type for argument named alignedSide
            String alignedSide) {
        int return2 = 0;

        int minLeftDistance = 100;
        int minRightDistance = 100;


        for (int count = 0; count < 10; count++) {
            if (minLeftDistance > leftDistance.getDistance(DistanceUnit.CM))
            {
                minLeftDistance = (int) leftDistance.getDistance(DistanceUnit.CM);
            }
            if (minRightDistance > rightDistance.getDistance(DistanceUnit.CM))
            {
                minRightDistance = (int) rightDistance.getDistance(DistanceUnit.CM);
            }
        }

        if (alignedSide == "left") {
            if (minLeftDistance <= 70) {
                return2 = 3;
            } else if (minRightDistance <= 70) {
                return2 = 2;
            } else {
                return2 = 1;
            }
        } else if (alignedSide == "right") {
            if (minLeftDistance <= 70) {
                return2 = 2;
            } else if (minRightDistance <= 70) {
                return2 = 1;
            } else {
                return2 = 3;
            }
        }

        return return2;
    }
    /** [START] CLASS VARIABLES GO HERE. THEY CAN BE USED IN ANY FUNCTION BELOW THIS POINT. **/
//    private String rawValue = "-1";

    // [START] webcam barcode scanner variables
//    private OpenCvWebcam webcam;
//    private InputImage image;
//    private BarcodeScannerOptions options =
//            new BarcodeScannerOptions.Builder()
//                    .setBarcodeFormats(Barcode.FORMAT_QR_CODE)
//                    .build();
//    private BarcodeScanner scanner = BarcodeScanning.getClient(options);
    // [END] webcam barcode scanner variables

//    private DcMotor lift;
//    private TouchSensor magLo;
//    private CRServo intake;
    private Servo rightSpike;
    private Servo leftSpike;
    private DistanceSensor leftDistance;
    private DistanceSensor rightDistance;
//    private TouchSensor magHi;

//    int autoLiftCone = 1500;
//    int autoLiftLo = 2322;
//    int autoLiftMed = 4350;
//    int autoLiftHi = 5600;
//    int autoLiftStack = 584;
//    boolean liftDown = false;

//    private int currentHeight;
    /** [END] CLASS VARIABLES GO HERE. THEY CAN BE USED IN ANY FUNCTION BELOW THIS POINT. **/

    @Override
    public void runOpMode()
    {
        leftDistance = hardwareMap.get(DistanceSensor.class, "leftDistance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "rightDistance");
        /** [START] LOCAL VARIABLES GO HERE. THEY CAN BE USED ONLY IN THIS FUNCTION. **/
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        /** [END] LOCAL VARIABLES GO HERE. THEY CAN BE USED ONLY IN THIS FUNCTION. **/

        /** [START] THIS IS OUR INIT CODE. IT RUNS WHEN THE "INIT" BUTTON IS PRESSED. **/
//        lift = hardwareMap.get(DcMotor.class, "lift");
//        intake = hardwareMap.get(CRServo.class, "intake");
        leftSpike = hardwareMap.get(Servo.class, "leftSpike" );
        rightSpike = hardwareMap.get(Servo.class, "rightSpike");
//        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /** This starts the webcam barcode scanner. This code takes up a lot of processing power **/
        /** so we will stop it once we start moving and don't need it to be running any more. **/
//        startWebcam();

        telemetry.addLine("Waiting for start");
        telemetry.update();

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
//            if (!liftDown) {
//                liftDown = true;
//                while (!magLo.isPressed()) {
//                    lift.setPower(-0.15);
//                }
//                lift.setPower(0);
//                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }
//            drive.setPoseEstimate(new Pose2d(0, 0, 0));
            // Turn on the intake
//            intake.setPower(-1);
            // Wait for the intake to start moving
//            sleep(2000);
//             Set our "auto" lift height to the "cone" position so our camera can see
//            currentHeight = autoLiftMed;
//             Move the lift to the "cone" position
//            doAutoLift(false);

            // down
            leftSpike.setPosition(0);
            // up
//        leftSpike.setPosition(1);
            //    up
//        rightSpike.setPosition(0);
            // down
            rightSpike.setPosition(1);

            int numValue = choiceFunction("left");
            telemetry.addData("numValue", numValue);
            telemetry.update();

            sleep(3000);

            /* This is our main AUTO code... */
//            This figures out what parking space to go to.
            if (1 == numValue)
            {
                telemetry.addLine("Going RIGHT");
                telemetry.update();
                goRight(drive);
            }
            else if (2 == numValue)
            {
                telemetry.addLine("Going CENTER");
                telemetry.update();
                goCenter(drive);
            }
            else
            {
                telemetry.addLine("Going LEFT");
                telemetry.update();
                goLeft(drive);
            }
            /** Move the lift back to the ground so our tele-op mode starts with the lift all **/
            /** the way down. THIS IS CRITICAL OR OUR AUTO HI BUTTON WILL BREAK THE LIFT!!! **/

//            currentHeight = 0;
//            doAutoLift(true);

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
    private void goLeft(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(-40, -64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

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
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(48,-29, Math.toRadians(0)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(52, -12, Math.toRadians(270)))
                .build();


        drive.followTrajectorySequence(trajSeq1);

        rightSpike.setPosition(0);

        drive.followTrajectorySequence(trajSeq2);

        leftSpike.setPosition(1);

        sleep(500);
    }

    private void goCenter(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(-40, -64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

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
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(48,-41, Math.toRadians(0)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(52, -18, Math.toRadians(270)))
                .build();

        drive.followTrajectorySequence(trajSeq1);

        rightSpike.setPosition(0);

        drive.followTrajectorySequence(trajSeq2);

        leftSpike.setPosition(1);

        sleep(500);

    }

    private void goRight(SampleMecanumDrive drive) {
        Pose2d startPose = new Pose2d(-40, -64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

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
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(48,-41, Math.toRadians(0)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(52, -12, Math.toRadians(270)))
                .build();


        drive.followTrajectorySequence(trajSeq1);

        rightSpike.setPosition(0);

        drive.followTrajectorySequence(trajSeq2);

        leftSpike.setPosition(1);

        sleep(500);
    }

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

} // END class autoPowerPlay
