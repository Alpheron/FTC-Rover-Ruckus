package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;


@Autonomous(name = "Distance_Test", group = "Concept")

public class Distance_Test_With_Brake extends LinearOpMode {
    BNO055IMU imu;



    private ElapsedTime runtime = new ElapsedTime();


    private static DcMotor frontleftmotor = null;
    private static DcMotor frontrightmotor = null;
    private static DcMotor backleftmotor = null;
    private static DcMotor backrightmotor = null;
    double frontleftpower = 0.5;
    double backleftpower = 0.5;
    double frontrightpower = 0.5;
    double backrightpower = 0.5;




    @Override
    public void runOpMode() {
        frontleftmotor = hardwareMap.get(DcMotor.class, "frontleftmotor");
        frontrightmotor = hardwareMap.get(DcMotor.class, "frontrightmotor");
        backrightmotor = hardwareMap.get(DcMotor.class, "backrightmotor");
        backleftmotor = hardwareMap.get(DcMotor.class, "backleftmotor");
        //frontleftmotor.setDirection(DcMotor.Direction.REVERSE);
        //backleftmotor.setDirection(DcMotor.Direction.REVERSE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
double ramp;
double rampd;
        frontleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
    
    
    runtime.reset();
    while(runtime.seconds() < 2){
    ramp = runtime.seconds()/2;
    if(ramp>.25){
        ramp = .25;
    }
        frontrightmotor.setPower(frontrightpower*ramp);
        frontleftmotor.setPower(-frontleftpower*ramp);
        backrightmotor.setPower(backrightpower*ramp);
        backleftmotor.setPower(-backleftpower*ramp);
        
    }
    runtime.reset();
    while(runtime.seconds()<2){
        rampd = 2-runtime.seconds();
        ramp = rampd/2;
    if(ramp>.25){
        ramp = .25;
    }
        frontrightmotor.setPower(frontrightpower*ramp);
        frontleftmotor.setPower(-frontleftpower*ramp);
        backrightmotor.setPower(backrightpower*ramp);
        backleftmotor.setPower(-backleftpower*ramp);
        
    
}
}


}
