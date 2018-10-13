package org.boofcv.android.recognition;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.support.annotation.NonNull;

import org.boofcv.android.misc.MiscUtil;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import boofcv.abst.fiducial.*;
import boofcv.abst.fiducial.calib.CalibrationDetectorChessboard;
import boofcv.abst.fiducial.calib.CalibrationDetectorSquareGrid;
import boofcv.abst.geo.calibration.DetectorFiducialCalibration;
import boofcv.alg.distort.LensDistortionOps;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.android.ConvertBitmap;
import boofcv.android.VisualizeImageData;
import boofcv.android.gui.VideoImageProcessing;
import boofcv.core.image.ConvertImage;
import boofcv.struct.calib.CameraPinholeRadial;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.ImageBase;
import boofcv.struct.image.ImageType;
import boofcv.struct.image.Planar;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.EulerType;
import georegression.struct.point.Point2D_F32;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Quaternion_F64;
import georegression.transform.se.SePointOps_F64;

//-----
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.List;

import static java.lang.Math.PI;


/**
 * Created by will on 4/04/17.
 */
class FiducialImageProcessor<T extends ImageBase> extends VideoImageProcessing<Planar<GrayU8>> {
    private FiducialSquareActivity fiducialSquareActivity;
    T input;

    boofcv.abst.fiducial.FiducialDetector<T> detector;

    Paint paintSelected = new Paint();
    Paint paintLine0 = new Paint();
    Paint paintLine1 = new Paint();
    Paint paintLine2 = new Paint();
    Paint paintLine3 = new Paint();
    private Paint textPaint = new Paint();
    private Paint textBorder = new Paint();

    Rect bounds = new Rect();

    protected FiducialImageProcessor(FiducialSquareActivity fiducialSquareActivity) {
        super(ImageType.pl(3, GrayU8.class));
        this.fiducialSquareActivity = fiducialSquareActivity;

        paintSelected.setColor(Color.argb(0xFF / 2, 0xFF, 0, 0));

        paintLine0.setColor(Color.RED);
        paintLine0.setStrokeWidth(4f);
        paintLine0.setFlags(Paint.ANTI_ALIAS_FLAG);
        paintLine1.setColor(Color.WHITE);
        paintLine1.setStrokeWidth(4f);
        paintLine1.setFlags(Paint.ANTI_ALIAS_FLAG);
        paintLine2.setColor(Color.BLUE);
        paintLine2.setStrokeWidth(4f);
        paintLine2.setFlags(Paint.ANTI_ALIAS_FLAG);
        paintLine3.setColor(Color.GREEN);
        paintLine3.setStrokeWidth(4f);
        paintLine3.setFlags(Paint.ANTI_ALIAS_FLAG);

        // Create out paint to use for drawing
        textPaint.setARGB(255, 255, 100, 100);
        textPaint.setTextSize(20);

        textBorder.setARGB(255, 0, 0, 0);
        textBorder.setTextSize(20);
        textBorder.setStyle(Paint.Style.STROKE);
        textBorder.setStrokeWidth(3);
    }

    @Override
    protected void declareImages(int width, int height) {
        super.declareImages(width, height);                     // VideoImageProcessing.declareImages --> declares the output images and 'storage' for them

        fiducialSquareActivity.intrinsic = MiscUtil.checkThenInventIntrinsic();
    }

    @Override
    protected void process(Planar<GrayU8> color, Bitmap output, byte[] storage) {
        if (fiducialSquareActivity.changed && fiducialSquareActivity.intrinsic != null) {   // (1) user has changed the slider control for thresholding  and  (2) intrinsic parameters are known
            fiducialSquareActivity.changed = false;
            System.out.println("FiducialImageProcessor: process: fiducialSquareActivity.createDetector()");
            detector = (boofcv.abst.fiducial.FiducialDetector) fiducialSquareActivity.createDetector();     // FiducialSquareBinaryActivity.createDetector --> FiducialDetectionManager.createDetector
            detector.setLensDistortion(LensDistortionOps.transformPoint(fiducialSquareActivity.intrinsic)); // CameraPinholeRadial model of instrinsic parameters
            if (input == null || input.getImageType() != detector.getInputType()) {         // input : <T extends ImageBase> --> input is one of GrayU8, GrayU16, etc
                input = detector.getInputType().createImage(1, 1);
            }
        }

        if (detector == null) {
            return;
        }

        ImageType inputType = detector.getInputType();
        if (inputType.getFamily() == ImageType.Family.GRAY) {
            input.reshape(color.width, color.height);
            ConvertImage.average(color, (GrayU8) input);                                    // averages the colour channels
        } else {
            input = (T) color;
        }

        detector.detect(input);

        if (fiducialSquareActivity.showInput) {                                             // display the input image unchanged...
            ConvertBitmap.multiToBitmap(color, output, storage);
        } else {                                                                            // ... or display the checquerboard or binarised/thresholded input image as appropriate
            GrayU8 binary = null;
            if (detector instanceof CalibrationFiducialDetector) {
                DetectorFiducialCalibration a = ((CalibrationFiducialDetector) detector).getCalibDetector();
                if (a instanceof CalibrationDetectorChessboard) {
                    binary = ((CalibrationDetectorChessboard) a).getAlgorithm().getBinary();
                } else {
                    binary = ((CalibrationDetectorSquareGrid) a).getAlgorithm().getBinary();
                }
            } else {
                binary = ((SquareBase_to_FiducialDetector) detector).getAlgorithm().getBinary();    // here -
            }
            VisualizeImageData.binaryToBitmap(binary, false, output, storage);
        }

        Canvas canvas = new Canvas(output);

        for (int i = 0; i < detector.totalFound(); i++) {
            detector.getFiducialToCamera(i, fiducialSquareActivity.targetToCamera);

            double width = detector.getWidth(i);
            drawCube(detector.getId(i), fiducialSquareActivity.targetToCamera, fiducialSquareActivity.intrinsic, width, canvas);


//        new Thread(new ClientThread()).start();  //  https://stackoverflow.com/questions/22271207/socket-programming-with-python-server-and-android-client
//        connect();  // https://stackoverflow.com/questions/22271207/socket-programming-with-python-server-and-android-client
        }

        if (fiducialSquareActivity.drawText != null) {
            renderDrawText(canvas);
        }

    }


    //----------------------------------------------------------------------------------------------



//    //---------------------------------
    private Socket socket;

    private static final int SERVERPORT = 5000;
    private static final String SERVER_IP = "192.168.43.252";
    class ClientThread implements Runnable {


        @Override
        public void run() {
            try {
                InetAddress serverAddr = InetAddress.getByName(SERVER_IP);
                socket = new Socket(serverAddr, SERVERPORT);
            } catch (UnknownHostException e1) {
                e1.printStackTrace();
            } catch (IOException e1) {
                e1.printStackTrace();
            }
        }
    }

//    //---------------------------------
//    //   https://stackoverflow.com/questions/22271207/socket-programming-with-python-server-and-android-client
    Socket client = null;
    PrintWriter printwriter;
    boolean connection = false;
    public void connect(String message_) {
        class Bob
            implements Runnable{
            String message;

            public Bob(String message_) {
                this.message = message_;
            }

            private void sendMessage() throws IOException {
                printwriter = new PrintWriter(client.getOutputStream(), true);
                printwriter.write("HI : "+message); // write the message to output stream
                printwriter.flush();
                printwriter.close();
                connection = true;
                System.out.println("socket" + "connected " + connection);

                if (client.isConnected()) {
                    System.out.println("Message sent");
                }
            }

            @Override
            public void run() {
                int numAttempts = 3;
                boolean messageSent = false;

                for(ii_ = 1 ; ii_ < numAttempts ; ii_++) {
                    try {

                        if (null == client) {
                            System.out.println("Connecting the socket");
                            client = new Socket(SERVER_IP, SERVERPORT);
                        }
                        if (!client.isConnected()) {
                            System.out.println("Re-connecting the socket");
                            client = new Socket(SERVER_IP, SERVERPORT);
                        }
                        sendMessage();
                        System.out.println("Message sent");
                        messageSent = true;
                        ii_ = numAttempts + 1;
                    } catch (UnknownHostException e2) {
                        System.out.println("Error: UnknownHostException: RUN  rosrun vos_aa1 test_python_socket_server.py " + e2.getMessage());
                        client = null;
                        e2.printStackTrace();
                    } catch (IOException e1) {
                        System.out.println("Error: IOException: RUN  rosrun vos_aa1 test_python_socket_server.py " + e1.getMessage());
                        client = null;
                        e1.printStackTrace();
                    }
                }
                if(!messageSent ) {
                    System.out.println("ERROR: Could not send message: " + message);
                }

            }
        }
        Thread t = new Thread(new Bob(message_));
        t.start();
    }

//    //---------------------------------
    //----------------------------------------------------------------------------------------------






    /**
     * Draws a flat cube to show where the square fiducial is on the image
     */
    public void drawCube(long number, Se3_F64 targetToCamera, CameraPinholeRadial intrinsic, double width,
                         Canvas canvas) {
        double r = width / 2.0;
        Point3D_F64 corners[] = new Point3D_F64[8];
        corners[0] = new Point3D_F64(-r, -r, 0);
        corners[1] = new Point3D_F64(r, -r, 0);
        corners[2] = new Point3D_F64(r, r, 0);
        corners[3] = new Point3D_F64(-r, r, 0);
        corners[4] = new Point3D_F64(-r, -r, r);
        corners[5] = new Point3D_F64(r, -r, r);
        corners[6] = new Point3D_F64(r, r, r);
        corners[7] = new Point3D_F64(-r, r, r);

        Point2D_F32 pixel[] = new Point2D_F32[8];
        Point2D_F64 p = new Point2D_F64();
        for (int i = 0; i < 8; i++) {
            Point3D_F64 c = corners[i];
            SePointOps_F64.transform(targetToCamera, c, c);
            PerspectiveOps.convertNormToPixel(intrinsic, c.x / c.z, c.y / c.z, p);
            pixel[i] = new Point2D_F32((float) p.x, (float) p.y);
        }

        Point3D_F64 centerPt = new Point3D_F64();

        SePointOps_F64.transform(targetToCamera, centerPt, centerPt);
        PerspectiveOps.convertNormToPixel(intrinsic,
                centerPt.x / centerPt.z, centerPt.y / centerPt.z, p);
        Point2D_F32 centerPixel = new Point2D_F32((float) p.x, (float) p.y);

        // red
        drawLine(canvas, pixel[0], pixel[1], paintLine0);
        drawLine(canvas, pixel[1], pixel[2], paintLine0);
        drawLine(canvas, pixel[2], pixel[3], paintLine0);
        drawLine(canvas, pixel[3], pixel[0], paintLine0);

        // black
        drawLine(canvas, pixel[0], pixel[4], paintLine1);
        drawLine(canvas, pixel[1], pixel[5], paintLine1);
        drawLine(canvas, pixel[2], pixel[6], paintLine1);
        drawLine(canvas, pixel[3], pixel[7], paintLine1);

        drawLine(canvas, pixel[4], pixel[5], paintLine2);
        drawLine(canvas, pixel[5], pixel[6], paintLine2);
        drawLine(canvas, pixel[6], pixel[7], paintLine2);
        drawLine(canvas, pixel[7], pixel[4], paintLine3);

        String numberString = "" + number;

        textPaint.getTextBounds(numberString, 0, numberString.length(), bounds);

        int textLength = bounds.width();
        int textHeight = bounds.height();

        canvas.drawText(numberString, centerPixel.x - textLength / 2, centerPixel.y + textHeight / 2, textBorder);
        canvas.drawText(numberString, centerPixel.x - textLength / 2, centerPixel.y + textHeight / 2, textPaint);


        String message_ = numberString+": at pixel ("+(centerPixel.x - textLength / 2)+","+(centerPixel.y + textHeight / 2)+")";
//        System.out.println("Before send "+message_);
//        System.out.flush();
//        connect(message_);
//        System.out.println("After send "+message_);
//        System.out.println("Before send "+message_);
//        System.out.flush();
//        connect(message_);
//        System.out.println("After send "+message_);

        Se3_F64 cameraToTarget = null;
        cameraToTarget = targetToCamera.invert(cameraToTarget);
        message_ = toMessage(numberString, cameraToTarget);
        System.out.println("Before send "+message_);
        System.out.flush();
        connect(message_);
        System.out.println("After send "+message_);


        textPaint.setARGB(255, 255, 100, 100);
        textPaint.setTextSize(20);
        String string = "x=" + String.valueOf(cameraToTarget.getX()).substring(0, 5);
        canvas.drawText(string, centerPixel.x - textLength / 2, centerPixel.y + textHeight + 10, textPaint);
        string = "y=" + String.valueOf(-1*cameraToTarget.getY()).substring(0, 5);
        canvas.drawText(string, centerPixel.x - textLength / 2, centerPixel.y + textHeight * 2 + 10, textPaint);
        string = "z=" + String.valueOf(-1*cameraToTarget.getZ()).substring(0, 5);
        canvas.drawText(string, centerPixel.x - textLength / 2, centerPixel.y + textHeight * 3 + 10, textPaint);
        ii_++;
        Quaternion_F64 detection_quat;
        System.out.println("ii_="+ii_+";  %optical: z=distance AND ");
        System.out.println("xyz_camera_FLU(:,ii_)=["+cameraToTarget.getZ()+" , "+(0 - cameraToTarget.getX())+" , "+(0 - cameraToTarget.getY())+"] ;  %optical: z=distance");
        detection_quat = new Quaternion_F64();
        ConvertRotation3D_F64.matrixToQuaternion(cameraToTarget.getRotation(), detection_quat);
        System.out.println("rotation_quat_camera_FLU(ii_)=Quaternion(["+detection_quat.w+", "+detection_quat.x+", "+detection_quat.y+", "+detection_quat.z+"]); %optical: z=distance");  //s <vx, vy, vz>

        textPaint.setARGB(255, 100, 100, 255);
        textPaint.setTextSize(25);
        string = "x=" + String.valueOf(cameraToTarget.getZ()).substring(0, 5);
        canvas.drawText(string, centerPixel.x - textLength / 2, centerPixel.y + textHeight * 4 + 10, textPaint);
        string = "y=" + String.valueOf(0 - cameraToTarget.getX()).substring(0, 5);
        canvas.drawText(string, centerPixel.x - textLength / 2, centerPixel.y + textHeight * 5 + 10, textPaint);
        string = "z=" + String.valueOf(0 - cameraToTarget.getY()).substring(0, 5);
        canvas.drawText(string, centerPixel.x - textLength / 2, centerPixel.y + textHeight * 6 + 10, textPaint);
        System.out.println("xyz_marker_RDF(:,ii_)=["+cameraToTarget.getZ()+" , "+(0 - cameraToTarget.getX())+" , "+(0 - cameraToTarget.getY())+"] ;  %optical: z=distance");
        detection_quat = new Quaternion_F64();
        ConvertRotation3D_F64.matrixToQuaternion(cameraToTarget.getRotation(), detection_quat);
        System.out.println("rotation_quat_marker_RDF(ii_)=Quaternion(["+detection_quat.w+", "+detection_quat.x+", "+detection_quat.y+", "+detection_quat.z+"]); %optical: z=distance");  //s <vx, vy, vz>
    }

    @NonNull
    private String toMessage(String numberString, Se3_F64 cameraToTarget) {
        Vector3D_F64 transl = cameraToTarget.getT();
        DenseMatrix64F rot = cameraToTarget.getRotation();
        String message_= numberString+": ";
        message_=message_+" transl=["+transl.getX()+","+transl.getY()+","+transl.getZ()+"] ";
        message_=message_+" rot=["+
            rot.get(0,0) +", "+rot.get(1,0)+", "+rot.get(2,0)+"; "+ //get(row, column): BoofCV is row-major, so the first row becomes the first column
            rot.get(0,1) +", "+rot.get(1,1)+", "+rot.get(2,1)+"; "+
            rot.get(0,2) +", "+rot.get(1,2)+", "+rot.get(2,2)+" ] ";
        return message_;
    }

    private static long ii_ = 0;

    private void renderDrawText(Canvas canvas) {
        textPaint.getTextBounds(fiducialSquareActivity.drawText, 0, fiducialSquareActivity.drawText.length(), bounds);

        int textLength = bounds.width();
        int textHeight = bounds.height();

        int x0 = canvas.getWidth() / 2 - textLength / 2;
        int y0 = canvas.getHeight() / 2 + textHeight / 2;

        canvas.drawText(fiducialSquareActivity.drawText, x0, y0, textBorder);
        canvas.drawText(fiducialSquareActivity.drawText, x0, y0, textPaint);
    }

    private void drawLine(Canvas canvas, Point2D_F32 a, Point2D_F32 b, Paint color) {
        canvas.drawLine(a.x, a.y, b.x, b.y, color);
    }


//
//    private void invoke(int tag_id, Se3_F64 targetToSensor_boofcvFrame) {
//
//
//        System.out.println("onCameraFrame: 3D Location: start for this tag "+tag_id+"---------------------------------------------------------");
//        Vector3D_F64 transBoofCV_TtoS = targetToSensor_boofcvFrame.getTranslation();
//        Quaternion_F64 quatBoofCV_TtoS = transform_to_quaternion_boofcv(targetToSensor_boofcvFrame);
//        System.out.println("onCameraFrame: 3D Location: targetToSensor_boofcvFrame : BoofCV frame "
//                +tag_id+": x = " + transBoofCV_TtoS.getX() + ", y = " + transBoofCV_TtoS.getY() + ", z = " + transBoofCV_TtoS.getZ());
//        System.out.println("onCameraFrame: 3D Location: targetToSensor_boofcvFrame : BoofCV frame "
//                +tag_id+": qx = " + quatBoofCV_TtoS.x + ", qy = " + quatBoofCV_TtoS.y + ", qz = " + quatBoofCV_TtoS.z + ", qw = " + quatBoofCV_TtoS.w);
//        double x = transBoofCV_TtoS.getX();  double y = transBoofCV_TtoS.getY();  double z = transBoofCV_TtoS.getZ();
//        double qx = quatBoofCV_TtoS.x;  double qy = quatBoofCV_TtoS.y;  double qz = quatBoofCV_TtoS.z;  double qw = quatBoofCV_TtoS.w;
//        System.out.println("onCameraFrame: 3D Location: targetToSensor_boofcvFrame : BoofCV frame "
//                +tag_id+": figure(f1); temp__plot_boofcv_StoT_inVOS_rot("+qw+","+qx+","+qy+","+qz+",  "+x+","+y+","+z+")  % "+tag_id);
//
//        DenseMatrix64F rot = targetToSensor_boofcvFrame.getR();
//        System.out.println("onCameraFrame: 3D Location: sensorToTarget_boofcvFrame : BoofCV frame : "+tag_id+": rotation matrix= [ "+
//                rot.get(0,0) +", "+rot.get(0,1)+", "+rot.get(0,2)+"; "+ //get(row, column)
//                rot.get(1,0) +", "+rot.get(1,1)+", "+rot.get(1,2)+"; "+
//                rot.get(2,0) +", "+rot.get(2,1)+", "+rot.get(2,2)+" ]");
//        System.out.println("onCameraFrame: 3D Location: sensorToTarget_boofcvFrame : BoofCV frame : "+tag_id+": rotation matrix= "
//                +targetToSensor_boofcvFrame.getR());
//
//        //  % 1) see if we can get rot, trans of inverted transform straight from BoofCV
//        Se3_F64 sensorToTarget_boofcvFrame_ = targetToSensor_boofcvFrame.invert(null);
//        Vector3D_F64 transBoofCV_StoT_ = sensorToTarget_boofcvFrame_.getTranslation();
//        System.out.println("onCameraFrame: 3D Location: targetToSensor_boofcvFrame : BoofCV frame "
//                +tag_id+": transBoofCV_StoT_: x = " + transBoofCV_StoT_.getX() + ", y = " + transBoofCV_StoT_.getY() + ", z = " + transBoofCV_StoT_.getZ());
//        DenseMatrix64F rotBoofCV_StoT_ = sensorToTarget_boofcvFrame_.getR();
//        System.out.println("onCameraFrame: 3D Location: sensorToTarget_boofcvFrame : BoofCV frame : "+tag_id+": rotation matrix rotBoofCV_StoT_= [ "+
//                rotBoofCV_StoT_.get(0,0) +", "+rotBoofCV_StoT_.get(0,1)+", "+rotBoofCV_StoT_.get(0,2)+"; "+ //get(row, column)
//                rotBoofCV_StoT_.get(1,0) +", "+rotBoofCV_StoT_.get(1,1)+", "+rotBoofCV_StoT_.get(1,2)+"; "+
//                rotBoofCV_StoT_.get(2,0) +", "+rotBoofCV_StoT_.get(2,1)+", "+rotBoofCV_StoT_.get(2,2)+" ]");
//
//            /*-----------------------------------------------------------------------------*/
//
//            /*-----------------------------------------------------------------------------*/
//
//        Se3_F64 sensorToTarget_boofcvFrame = targetToSensor_boofcvFrame.invert(null);
//        Vector3D_F64 sensorToTarget_boofcvFrame_transl = sensorToTarget_boofcvFrame.getTranslation();
//        Quaternion_F64 sensorToTarget_boofcvFrame_quat = transform_to_quaternion_boofcv(sensorToTarget_boofcvFrame);
//        System.out.println("onCameraFrame: 3D Location: sensorToTarget_boofcvFrame : BoofCV frame "
//                +tag_id+": x = " + sensorToTarget_boofcvFrame_transl.getX() + ", y = " + sensorToTarget_boofcvFrame_transl.getY() + ", z = " + sensorToTarget_boofcvFrame_transl.getZ());
//        System.out.println("onCameraFrame: 3D Location: sensorToTarget_boofcvFrame : BoofCV frame "
//                +tag_id+": qx = " + sensorToTarget_boofcvFrame_quat.x + ", qy = " + sensorToTarget_boofcvFrame_quat.y + ", qz = " + sensorToTarget_boofcvFrame_quat.z + ", qw = " + sensorToTarget_boofcvFrame_quat.w);
//
//        System.out.println("onCameraFrame: 3D Location: end for this tag "+tag_id+"---------------------------------------------------------");
//
//        // testing 2017_08_23
//        double[] eulerZYZ = new double[]{0, 0, 0};
//        ConvertRotation3D_F64.matrixToEuler(targetToSensor_boofcvFrame.getR(), EulerType.ZYZ, eulerZYZ);
////        Log.i(logTagTag, "onCameraFrame : testing 2017_08_23: eulerZYZ = " + eulerZYZ[0] + "," + eulerZYZ[1] + "," + eulerZYZ[2]);
//
//        Se3_F64 sensorToTarget_testing = null;
//        sensorToTarget_testing = targetToSensor_boofcvFrame.invert(sensorToTarget_testing);
//
//        Quaternion_F64 sensorToTarget_testing_quat;
//        sensorToTarget_testing_quat = new Quaternion_F64();
//        ConvertRotation3D_F64.matrixToQuaternion(sensorToTarget_testing.getRotation(), sensorToTarget_testing_quat);
////
////                            detectedFeaturesClient.reportDetectedFeature(80000+tag_id,
////                                    sensorToTarget_testing.getZ(), sensorToTarget_testing.getX(), sensorToTarget_testing.getY(),
////                                    sensorToTarget_testing_quat.z,sensorToTarget_testing_quat.x,sensorToTarget_testing_quat.y,sensorToTarget_testing_quat.w);
//
//        /** Mirror across YZ plane / mirror along X axis:
//         * sensor-to-target +x = target-to-sensor +z
//         * sensor-to-target +y = target-to-sensor +x
//         * sensor-to-target +z = target-to-sensor +y   */
//        Se3_F64 targetToSensor_boofcvFrame_testing = new Se3_F64();
////        detector.getFiducialToCamera(detectionOrder_, targetToSensor_boofcvFrame_testing);
//        targetToSensor_boofcvFrame_testing = targetToSensor_boofcvFrame ;
//        /** Convert from BoofCV coordinate convention to ROS coordinate convention */
//        Se3_F64 targetToSensor_ROSFrame = new Se3_F64();
//        targetToSensor_ROSFrame.setTranslation(new Vector3D_F64(
//                targetToSensor_boofcvFrame_testing.getZ(), targetToSensor_boofcvFrame_testing.getX(), targetToSensor_boofcvFrame_testing.getY()));
//
//
//        Quaternion_F64 tToS_Boof_testing_quat = ConvertRotation3D_F64.matrixToQuaternion(targetToSensor_boofcvFrame_testing.getRotation(), null);
//        Quaternion_F64 tToS_ROS_testing_quat = new Quaternion_F64(  /** new Quaternion_F64(w, x, y, z) */
//                tToS_Boof_testing_quat.w, tToS_Boof_testing_quat.z, tToS_Boof_testing_quat.x, tToS_Boof_testing_quat.y);
//        Quaternion_F64 sensorToTarget_ROSFrame_mirrored_q = new Quaternion_F64(  /** new Quaternion_F64(w, x, y, z) */
//                tToS_ROS_testing_quat.w, tToS_ROS_testing_quat.x, -tToS_ROS_testing_quat.y, -tToS_ROS_testing_quat.z
//        );
//        DenseMatrix64F sensorToTarget_ROSFrame_mirrored_rot = quaternionToNewMAtrix(sensorToTarget_ROSFrame_mirrored_q);
//
//        /** Mirror across the XY plane. */
//        Se3_F64 sensorToTarget_ROSFrame_mirrored = new Se3_F64();
//        sensorToTarget_ROSFrame_mirrored.setTranslation(new Vector3D_F64(
//                targetToSensor_ROSFrame.getX(), -targetToSensor_ROSFrame.getY(), -targetToSensor_ROSFrame.getZ()));
//
//        sensorToTarget_ROSFrame_mirrored.setRotation(sensorToTarget_ROSFrame_mirrored_rot);
//
//        DenseMatrix64F sensorToTarget_ROSFrame_mirrored_rot_rotate_around_X_by_180_rot = new DenseMatrix64F(3, 3);
//
//        DenseMatrix64F rotate_around_X_by_180 = CommonOps.identity(3);
//        ConvertRotation3D_F64.setRotX(PI, rotate_around_X_by_180);
////                        CommonOps.mult(rotate_around_X_by_180,sensorToTarget_ROSFrame_mirrored_rot,sensorToTarget_ROSFrame_mirrored_rot_rotate_around_X_by_180_rot);
//        /** Note: post-multiply with BoofCV - I think that it is column-major ?? */  // TODO - check BoofCV conventions
//        CommonOps.mult(sensorToTarget_ROSFrame_mirrored_rot, rotate_around_X_by_180, sensorToTarget_ROSFrame_mirrored_rot_rotate_around_X_by_180_rot);
//        Quaternion_F64 sensorToTarget_ROSFrame_mirrored_rotate_around_X_by_180_q = new Quaternion_F64();
//        ConvertRotation3D_F64.matrixToQuaternion(sensorToTarget_ROSFrame_mirrored_rot_rotate_around_X_by_180_rot, sensorToTarget_ROSFrame_mirrored_rotate_around_X_by_180_q);
//
//
////                      /** This is the camera-to-marker transform */ - keep this code, but publish the camera-to-robot-base transform instead
////                      /** Report as e.g. 60170, 60155, etc. */
////        for(VisionTask visionTaskInList : visionTaskListToExecute) {
////            detectedFeaturesClient.reportDetectedFeature(visionTaskInList.getRobotId(), visionTaskInList.getRequestId(),
////                    60000 + tag_id,
////                    // TODO - use this - int tag_id_reported = MARKER_OFFSET_INT+tag_id;
////                    sensorToTarget_ROSFrame_mirrored.getX(), sensorToTarget_ROSFrame_mirrored.getY(), sensorToTarget_ROSFrame_mirrored.getZ(),
////                    sensorToTarget_ROSFrame_mirrored_rotate_around_X_by_180_q.x, sensorToTarget_ROSFrame_mirrored_rotate_around_X_by_180_q.y, sensorToTarget_ROSFrame_mirrored_rotate_around_X_by_180_q.z, sensorToTarget_ROSFrame_mirrored_rotate_around_X_by_180_q.w);
////
////            if(RECORD_DATA) {
////                Log.i(logTagTag, "detectAndEstimate_BoofCV_Fiducial: recording detection data: start.");
////                printlnToFile("\ntag_id:"+tag_id+" [ "+pixelPosition.getU()+" , "+pixelPosition.getV()+" ] ", imageFrameTime);
////                Se3_F64 trans = sensorToTarget_ROSFrame_mirrored;
////                printlnToFile("task: {robotId:" + visionTaskInList.getRobotId()+ ", featureDescriptor: " + tag_id + " }, pose: { x: " + trans.getX() + ", y:" + trans.getY() + ", z:" + trans.getZ() + " }", imageFrameTime);
////                RECORD_DATA_RECORDED = true;
////                Log.i(logTagTag, "detectAndEstimate_BoofCV_Fiducial: recording detection data: end.");
////            } else {
////                Log.i(logTagTag, "detectAndEstimate_BoofCV_Fiducial: not recording detection data.");
////            }
////        }
//
//
//        Se3_F64 sensorToTarget_ROSFrame_mirrored_rot_rotate_around_X_by_180_t = new Se3_F64();
//        sensorToTarget_ROSFrame_mirrored_rot_rotate_around_X_by_180_t.setRotation(sensorToTarget_ROSFrame_mirrored_rot_rotate_around_X_by_180_rot);
//        sensorToTarget_ROSFrame_mirrored_rot_rotate_around_X_by_180_t.setTranslation(sensorToTarget_ROSFrame_mirrored.getTranslation());
//
//        VisionTask visionTask = new VisualizeImageData()
//        geometry_msgs.Point position = visionTask.getRelationToBase().getPosition();
//        Se3_F64 transformOfFeatureInVisualModel = new Se3_F64();                    // transform from robot to marker, e.g. base_link to feature
//        transformOfFeatureInVisualModel.setTranslation(position.getX(), position.getY(), position.getZ());
//        Quaternion_F64 rotationOfFeatureInVisualModel_q = convertRosToBoofcvQuaternion(visionTask);
//        DenseMatrix64F rotationOfFeatureInVisualModel_m = quaternionToNewMAtrix(rotationOfFeatureInVisualModel_q);
//        transformOfFeatureInVisualModel.setRotation(ConvertRotation3D_F64.quaternionToMatrix(rotationOfFeatureInVisualModel_q, rotationOfFeatureInVisualModel_m));
//        Se3_F64 transformOfFeatureInVisualModel_inv = new Se3_F64();
//        transformOfFeatureInVisualModel.invert(transformOfFeatureInVisualModel_inv); // transform from marker to robot
//
//        if(smartCameraExtrinsicsCalibrator_first_notification_this_frame || detect_all_markers_in_frame) {
//            smartCameraExtrinsicsCalibrator_first_notification_this_frame = false;
//
////                        public void recordRobotObservation(String robotId_, java.util.Date imageFrameTime_, PixelPosition pixelPosition_, Se3_F64 transformOfFeatureInVisualModel_)
//            Log.i(TAG,"before smartCameraExtrinsicsCalibrator.recordRobotObservation");
//            smartCameraExtrinsicsCalibrator.recordRobotObservation(
//                    singleDummyRobotId_.idString(), imageFrameTime, pixelPosition, transformOfFeatureInVisualModel);
//            Log.i(TAG,"after smartCameraExtrinsicsCalibrator.recordRobotObservation");
//            Log.i(TAG,"before smartCameraExtrinsicsCalibrator.estimateExtrinsicsFromObservations()");
//            Se3_F64 extrinsics = smartCameraExtrinsicsCalibrator.estimateExtrinsicsFromObservations(matGray.width(), matGray.height());
//            Log.i(TAG,"after smartCameraExtrinsicsCalibrator.estimateExtrinsicsFromObservations(): extrinsics="+extrinsics);
//
//            //  askForPose_then_recordDetection(imageFrameTime, transformOfFeatureInVisualModel, pixelPosition);
//        }
//
//        DenseMatrix64F sensorToTarget_ROSFrame_toRobotBaseLink_rot = new DenseMatrix64F(3, 3);
//
//        Se3_F64 sensorToTarget_ROSFrame_toRobotBaseLink = new Se3_F64();
//
////                      this is the camera-to-robot-base transform
////                      /** Report as e.g. 60170, 60155, etc. */
//        transformOfFeatureInVisualModel_inv.concat(                                 // pose from previous as transform from camera to marker
//                sensorToTarget_ROSFrame_mirrored_rot_rotate_around_X_by_180_t,      // transform from marker to robot, from robot visual model
//                sensorToTarget_ROSFrame_toRobotBaseLink);                          // output
//        Quaternion_F64 sensorToTarget_ROSFrame_toRobotBaseLink_q = new Quaternion_F64();
//        ConvertRotation3D_F64.matrixToQuaternion(sensorToTarget_ROSFrame_toRobotBaseLink.getRotation(), sensorToTarget_ROSFrame_toRobotBaseLink_q);
//
//        for(VisionTask visionTaskInList: visionTaskListToExecute) {
//            detectedFeaturesClient.reportDetectedFeature(visionTaskInList.getRobotId(), visionTaskInList.getRequestId(),
//                    70000 + tag_id,
//                    sensorToTarget_ROSFrame_toRobotBaseLink.getX(), sensorToTarget_ROSFrame_toRobotBaseLink.getY(), sensorToTarget_ROSFrame_toRobotBaseLink.getZ(),
//                    sensorToTarget_ROSFrame_toRobotBaseLink_q.x, sensorToTarget_ROSFrame_toRobotBaseLink_q.y, sensorToTarget_ROSFrame_toRobotBaseLink_q.z, sensorToTarget_ROSFrame_toRobotBaseLink_q.w);
//        }
//        //  Report the detected feature pose in the world coordinate frame/system,
//        // applying the camera's current pose to the detected feature's pose,
//        // before reporting to the VOS Server.
//
//        if (poseKnown()) {
//            Log.i(logTagTag, "poseKnown()");
//            Log.i(logTagTag, "poseKnown(): translation: " + MainActivity.this.position[0] + "," + MainActivity.this.position[1] + "," + MainActivity.this.position[2]);
//            Log.i(logTagTag, "poseKnown(): rotation quaternion: " + MainActivity.this.orientation[0] + "," + MainActivity.this.orientation[1] + "," + MainActivity.this.orientation[2] + "," + MainActivity.this.orientation[3]);
//            Se3_F64 worldToCamera = new Se3_F64();
//            worldToCamera.setTranslation(MainActivity.this.position[0], MainActivity.this.position[1], MainActivity.this.position[2]);
//            Quaternion_F64 worldToCamera_rot_q = new Quaternion_F64(MainActivity.this.orientation[3], MainActivity.this.orientation[0], MainActivity.this.orientation[1], MainActivity.this.orientation[2]);
//            DenseMatrix64F worldToCamera_rot_m = new DenseMatrix64F(3, 3);
//            worldToCamera.setRotation(
//                    ConvertRotation3D_F64.quaternionToMatrix(
//                            worldToCamera_rot_q,  //  (double w, double x, double y, double z)
//                            worldToCamera_rot_m));
//
//            System.out.println("poseKnown: sending id=" + (40000 + tag_id)
//                    +", x="+worldToCamera.getX()+", y="+worldToCamera.getY()+",z="+ worldToCamera.getZ()
//                    +",  qx="+worldToCamera_rot_q.x+", qy="+worldToCamera_rot_q.y+", qz="+worldToCamera_rot_q.z+", qw="+worldToCamera_rot_q.w);
//
//            for(VisionTask visionTaskInList: visionTaskListToExecute) {
//                detectedFeaturesClient.reportDetectedFeature(visionTaskInList.getRobotId(), visionTaskInList.getRequestId(),
//                        40000 + tag_id,
//                        worldToCamera.getX(), worldToCamera.getY(), worldToCamera.getZ(),
//                        worldToCamera_rot_q.x, worldToCamera_rot_q.y, worldToCamera_rot_q.z, worldToCamera_rot_q.w);
//            }
//            Se3_F64 worldToRobotBaseLink = new Se3_F64();
//            sensorToTarget_ROSFrame_toRobotBaseLink.concat(
//                    worldToCamera,
//                    worldToRobotBaseLink
//            );
//            Quaternion_F64 worldToRobotBaseLink_q = new Quaternion_F64();
//            ConvertRotation3D_F64.matrixToQuaternion(worldToRobotBaseLink.getRotation(), worldToRobotBaseLink_q);
//
//            for(VisionTask visionTaskInList: visionTaskListToExecute) {
//                detectedFeaturesClient.reportDetectedFeature(visionTaskInList.getRobotId(), visionTaskInList.getRequestId(),
//                        50000 + tag_id,
//                        worldToRobotBaseLink.getX(), worldToRobotBaseLink.getY(), worldToRobotBaseLink.getZ(),
//                        worldToRobotBaseLink_q.x, worldToRobotBaseLink_q.y, worldToRobotBaseLink_q.z, worldToRobotBaseLink_q.w);
//
//                System.out.println("poseKnown: sending id=" + (50000 + tag_id)
//                        +", x="+worldToRobotBaseLink.getX()+", y="+worldToRobotBaseLink.getY()+",z="+ worldToRobotBaseLink.getZ()
//                        +",  qx="+worldToRobotBaseLink_q.x+", qy="+worldToRobotBaseLink_q.y+", qz="+worldToRobotBaseLink_q.z+", qw="+worldToRobotBaseLink_q.w);
//            }
//
//        } else {
////            Log.i(logTagTag, "! poseKnown()");
//        }
//
//
//        Se3_F64 translation_to_marker = sensorToTarget_ROSFrame_mirrored;
//        Quaternion_F64 quaternion_to_marker = sensorToTarget_ROSFrame_mirrored_rotate_around_X_by_180_q;
//
//        double[] eulerZYZ_fromInvert = new double[]{0, 0, 0};
//        ConvertRotation3D_F64.matrixToEuler(sensorToTarget_testing.getR(), EulerType.ZYZ, eulerZYZ_fromInvert);
////        Log.i(logTagTag, "onCameraFrame : testing 2017_08_23: eulerZYZ_fromInvert = " + eulerZYZ_fromInvert[0] + "," + eulerZYZ_fromInvert[1] + "," + eulerZYZ_fromInvert[2]);
//        Vector3D_F64 sensorToTarget_testing_trans = sensorToTarget_testing.getTranslation();
////        Log.i(logTagTag, "onCameraFrame: testing 2017_08_23: sensorToTarget_testing_trans : x = " + sensorToTarget_testing_trans.getX() + ", y = " + sensorToTarget_testing_trans.getY() + ", z = " + sensorToTarget_testing_trans.getZ());
//        sensorToTarget_testing_quat = new Quaternion_F64();
//        ConvertRotation3D_F64.matrixToQuaternion(sensorToTarget_testing.getR(), sensorToTarget_testing_quat);
////        Log.i(logTagTag, "onCameraFrame: testing 2017_08_23: sensorToTarget_testing_quat : qx = " + sensorToTarget_testing_quat.x + ", qy = " + sensorToTarget_testing_quat.y + ", qz = " + sensorToTarget_testing_quat.z + ", qw = " + sensorToTarget_testing_quat.w);
//
//        if (isPartOfRobotVisualModel(tag_id)) {
////            List<DetectedTag> visionTaskFeaturesDetected = visionTaskFeaturesDetected(robotsDetected_, singleDummyRobotId_);
////            DetectedTag detectedTag = new DetectedTag(tag_id, translation_to_marker, quaternion_to_marker);
////            visionTaskFeaturesDetected.add(detectedTag);
////            Log.i(logTagTag, "onCameraFrame: isPartOfRobotVisualModel TAG - tag_id " + tag_id + " - 2D Image Location = " + pixelPosition);
////        } else if (isALandmark(tag_id)) {
////            DetectedTag detectedTag = new DetectedTag(tag_id, translation_to_marker, new Point2D_F64(pixelPosition.getU(),pixelPosition.getV()));
////            landmarkFeatures.add(detectedTag);
////            Log.i(logTagTag, "onCameraFrame: isALandmark TAG - tag_id " + tag_id + " landmarkFeatures.size()=" + landmarkFeatures.size() + " - 2D Image Location = " + pixelPosition);
//        } else { // not part of something that we are looking for, so ignore
////            Log.i(logTagTag, "onCameraFrame: IGNORING TAG - not part of robot visual model - tag_id " + tag_id + " - 2D Image Location = " + pixelPosition);
////            myResult = true;
////            return this;
//            return
//        }
//
//        //// TODO - timing here  c[camera_num]-f[frameprocessed]-detectionOrder_[iteration]-t[tagid]
////        Log.i(logTagTag, "onCameraFrame: after detectedFeaturesClient.reportDetectedFeature");
////        if (LOCALISING_CAMERA_FROM_OBSERVED_FEATURES) {
////            updateLocationFromDetectedFeature(tag_id, logTagTag, translation_to_marker, quaternion_to_marker);
////        }
////        if (TESTING_TRANSFORMATIONS_OF_TRANSFORMS) {
////            variousUnusedAttemptsAtCoordinateSystemCorrection();
////        }
////        myResult = false;
////        return this;
//    }
//

    /* Dev: part of robot visual model */
    public static boolean isPartOfRobotVisualModel(final int tag_id) {
        return tag_id == 170 || tag_id == 250 || tag_id == 290 || tag_id == 330
                || tag_id == 1650
                || tag_id == 57 || tag_id == 157 || tag_id == 257 || tag_id == 357 || tag_id == 457 || tag_id == 557
                //|| tag_id == 210
                ;
    }

    @NonNull
    private Quaternion_F64 transform_to_quaternion_boofcv(Se3_F64 transform_boofcv) {
        DenseMatrix64F rotation_mat_boofcv = transform_boofcv.getR();
        return rotation_mat_to_quaternion_boofcv(rotation_mat_boofcv);
    }

    @NonNull
    private Quaternion_F64 rotation_mat_to_quaternion_boofcv(DenseMatrix64F rotation_mat_boofcv) {
        Quaternion_F64 quaternionBoofCV = new Quaternion_F64();
        ConvertRotation3D_F64.matrixToQuaternion(rotation_mat_boofcv, quaternionBoofCV);
        return quaternionBoofCV;
    }

    @NonNull
    private DenseMatrix64F quaternionToNewMAtrix(Quaternion_F64 quaternion) {
        DenseMatrix64F matrix = new DenseMatrix64F(3, 3);
        ConvertRotation3D_F64.quaternionToMatrix(quaternion, matrix);
        return matrix;
    }
}




