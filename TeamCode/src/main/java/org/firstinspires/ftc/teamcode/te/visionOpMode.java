package org.firstinspires.ftc.teamcode.te;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hwMecanum;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.hwMecanum;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.FtcDashboard;

import com.acmerobotics.dashboard.config.Config;

@Config
@Autonomous(name="VISIONTEST", group="Tutorials")

public class visionOpMode extends LinearOpMode{

    public OpenCvCamera webcam;
    private contourCentroidDetector pipeline;

    private double crThreshHigh = 220;
    private double crThreshLow = 120;
    private double cbThreshHigh = 80;
    private double cbThreshLow = 0;

    private int minRectangleArea = 2000;
    private double middleBarcodeRangeBoundary = 0.5; //i.e 30% of the way across the frame from the left
    private double rightBarcodeRangeBoundary = 0.6; //i.e 60% of the way achhross the frame from the left

    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 120.0, 0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 220,74);

    public int cameraMonitorViewId;
    public enum barcodePosition{
        LEFT,
        RIGHT,
        CENTER
    }
    public barcodePosition position;

    public void initVision(HardwareMap h){
        // OpenCV webcam
        int cameraMonitorViewId = h.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", h.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(h.get(WebcamName.class, "camera"), cameraMonitorViewId);
        //OpenCV Pipeline

        pipeline = new contourCentroidDetector(0.2, 0.2, 0.2, 0.2);

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);

        webcam.setPipeline(pipeline);

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {

            }
        });
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        FtcDashboard.getInstance().startCameraStream(webcam, 10);
    }



    public void checkTE(){
        if(pipeline.error){
            telemetry.addData("Exception: ", pipeline.debug.getStackTrace());
        }
        // Only use this line of the code when you want to find the lower and upper values, using Ftc Dashboard (https://acmerobotics.github.io/ftc-dashboard/gettingstarted)
        testing(pipeline);

        // Watch our YouTube Tutorial for the better explanation

        double rectangleArea = pipeline.getRectArea();

        //Print out the area of the rectangle that is found.
        telemetry.addData("Rectangle Area", rectangleArea);

        //Check to see if the rectangle has a large enough area to be a marker.
        if(rectangleArea > minRectangleArea){
            //Then check the location of the rectangle to see which barcode it is in.
            if(pipeline.getRectMidpointX() >200){
                telemetry.addData("Barcode Position", "Right");
                position=barcodePosition.RIGHT;
            }
            else if(pipeline.getRectMidpointX() < 200){
                telemetry.addData("Barcode Position", "Center");
                position=barcodePosition.CENTER;
            }

        }
        else {
            telemetry.addData("Barcode Position", "Left");
            position=barcodePosition.LEFT;
        }

        telemetry.update();
    }

    @Override
    public void runOpMode()
    {

        initVision(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            checkTE();
            testing(pipeline);
        }

    }

    public void testing(contourCentroidDetector pipeline){
        if(lowerRuntime + 0.05 < getRuntime()){
            crThreshLow += -gamepad1.left_stick_y;
            cbThreshLow += gamepad1.left_stick_x;
            lowerRuntime = getRuntime();
        }
        if(upperRuntime + 0.05 < getRuntime()){
            crThreshHigh += -gamepad1.right_stick_y;
            cbThreshHigh += gamepad1.right_stick_x;
            upperRuntime = getRuntime();
        }

        crThreshLow = inValues(crThreshLow, 0, 255);
        crThreshHigh = inValues(crThreshHigh, 0, 255);
        cbThreshLow = inValues(cbThreshLow, 0, 255);
        cbThreshHigh = inValues(cbThreshHigh, 0, 255);

        pipeline.configureScalarLower(0.0, crThreshLow, cbThreshLow);
        pipeline.configureScalarUpper(255.0, crThreshHigh, cbThreshHigh);

        telemetry.addData("lowerCr ", crThreshLow);
        telemetry.addData("lowerCb ", cbThreshLow);
        telemetry.addData("UpperCr ", crThreshHigh);
        telemetry.addData("UpperCb ", cbThreshHigh);
    }

    public double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }

    public barcodePosition getPosition() {
        return position;
    }

    public double getArea(){
        return pipeline.getRectArea();
    }

    public double getCenter(){
        return pipeline.getRectMidpointX();
    }

}