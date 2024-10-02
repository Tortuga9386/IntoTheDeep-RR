package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.subsystems.*;

import java.util.List;
import java.util.Locale;

/**
 * This is NOT an opmode.
 *
 * This class initiates all the specific hardware for a robot.
 * The various subsystems classes are located in the subsystem folder.
 * This code is used by all the other opmodes.
 *
 */
@Disabled
public class RobotBase extends OpMode
{
    //Define constants that should NOT be adjusted by manual calibration
    protected boolean           INITIALIZE_IMU      = true;
    protected boolean           INITIALIZE_DRIVE    = true;

    //Make subsystems available to all class extensions
    public Drive drive;
    public Lift lift;
    public SensorHuskyLens sensorHuskyLens;

    /* Constructor */
    public RobotBase(){ }

    /* Initialize standard Hardware interfaces */
    @Override
    public void init() {
        //Read calibration constants
        //new Calibration().readFromFile();

        //1 of 2 >> Initialize sensor bulk read, then after all the hardware maps are set below, enable BulkCachingMode.AUTO
//        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        //Initialize subsystems
        lift = new Lift(hardwareMap, this);
        sensorHuskyLens = new SensorHuskyLens(hardwareMap, this);


        //Initialize system
//        if (INITIALIZE_DRIVE) {
//            //Only create the requested drive object, may be different for TeleOp, AutoOp
//            Log.i("RobotBase.driveMethod", "DRIVE_METHOD.toString()");
//            try {
                drive = new Drive(hardwareMap, this);
//            } catch(Exception e) {
//                Log.i("RobotBase.driveMethod", "Drive failed to initialize");
//            }
//        } else {
//            Log.i("RobotBase.driveMethod", "INITIALIZE_DRIVE FALSE");
//        }

        //Initialize Webcam
//        if (INITIALIZE_WEBCAM) {
//            //Only create the requested drive object, may be different for TeleOp, AutoOp
//            Log.i("RobotBase.driveMethod", DRIVE_METHOD.toString());
//            try {
//                switch (WEBCAM_METHOD) {
//                    case WebcamOpenCV:
//                        webcam = new WebcamOpenCV(hardwareMap, this);
//                        break;
//                    case WebcamTF2023:
//                        webcam = new WebcamTF2023(hardwareMap, this);
//                        break;
//                    default:
//                        webcam = new WebcamTF2023(hardwareMap, this);
//                        Log.i("RobotBase.webcamMethod", "FAIL TO DEFAULT: WebcamTF2023");
//                }
//            } catch(Exception e) {
//                Log.i("RobotBase.webcamMethod", "Webcam failed to initialize");
//            }
//        } else {
//            Log.i("RobotBase.webcamMethod", "INITIALIZE_WEBCAM FALSE");
//        }

//        //Initialize IMU for angles
//        if (INITIALIZE_IMU) {
//
//            try {
//                //controlHub = new ControlHub(hardwareMap, this);
//                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//                parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
//                parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//                parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//                parameters.loggingEnabled      = true;
//                parameters.loggingTag          = "IMU";
//                parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//                //imu = hardwareMap.get(BNO055IMU.class, "imu");
//
//                //imu.initialize(parameters);
//            } catch(Exception e) {
//                Log.i("RobotBase.IMU", "IMU failed to initialize");
//            }
//        }
//
//        try {
//            //2 of 2 >> All the hardware maps are set above, now enable BulkCachingMode.AUTO on hubs
//            for (LynxModule hub : hubs) {
//                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//            }
//        } catch(Exception e) {
//            Log.i("RobotBase.IMU", "IMU bulk cache mode failed to initialize");
//        }
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        telemetry.clearAll();
    }

    public void loop() {
        //Do nothing, use these classes in the opModes

    }

    /*
     * Code to run when the op mode is first disabled goes here
     */
    @Override
    public void stop() {

    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    
 }
