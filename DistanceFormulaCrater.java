package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
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
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;


@Autonomous(name = "DistanceFormulaCrater", group = "Concept")

public class DistanceFormulaCrater extends LinearOpMode {
    BNO055IMU imu;



    private ElapsedTime runtime = new ElapsedTime();


    private static DcMotor frontleftmotor = null;
    private static DcMotor frontrightmotor = null;
    private static DcMotor backleftmotor = null;
    private static DcMotor backrightmotor = null;
    private static DcMotor hanging1 = null;
    private static DcMotor hanging2 = null;
    private static Servo teammarker = null;
    private DistanceSensor sensorRange;
    double frontleftpower = 0.4;
    double backleftpower = 0.4;
    double frontrightpower = 0.4;
    double backrightpower = 0.4;




    @Override
    public void runOpMode() {
        frontleftmotor = hardwareMap.get(DcMotor.class, "frontleftmotor");
        frontrightmotor = hardwareMap.get(DcMotor.class, "frontrightmotor");
        backrightmotor = hardwareMap.get(DcMotor.class, "backrightmotor");
        backleftmotor = hardwareMap.get(DcMotor.class, "backleftmotor");
        //sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        //frontleftmotor.setDirection(DcMotor.Direction.REVERSE);
        //backleftmotor.setDirection(DcMotor.Direction.REVERSE);
        hanging1 = hardwareMap.get(DcMotor.class, "hanging1");
        hanging2 = hardwareMap.get(DcMotor.class, "hanging2");
        hanging1.setDirection(DcMotor.Direction.REVERSE);
        teammarker = hardwareMap.get(Servo.class, "teammarker");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        //imu.initialize(parameters);
        teammarker.setPosition(0.0);
        waitForStart();
        //composeTelemetry();
        unhook();
        leftpath();
    
        
        
        
      
        
      
    }



    private void moveForward(long sleep){
        frontrightmotor.setPower(frontrightpower);
        frontleftmotor.setPower(-frontleftpower);
        backrightmotor.setPower(backrightpower);
        backleftmotor.setPower(-backleftpower);
        sleep(sleep);

    }

    private void moveBackward(long sleep){
        frontrightmotor.setPower(-frontrightpower);
        frontleftmotor.setPower(frontleftpower);
        backrightmotor.setPower(-backrightpower);
        backleftmotor.setPower(backrightpower);
        sleep(sleep);
    }


    private void spinleft(long sleep){
        frontrightmotor.setPower(frontrightpower);
        frontleftmotor.setPower(frontleftpower);
        backrightmotor.setPower(backrightpower);
        backleftmotor.setPower(backleftpower);
        sleep(sleep);
        

    }

    private void spinright(long sleep){
        frontrightmotor.setPower(-frontrightpower);
        frontleftmotor.setPower(-frontleftpower);
        backrightmotor.setPower(-backrightpower);
        backleftmotor.setPower(-backleftpower);
        sleep(sleep);
    }

    private void straferight(long sleep) {
        frontleftmotor.setPower(0.4);
        backleftmotor.setPower(-0.4);
        frontrightmotor.setPower(-0.4);
        backrightmotor.setPower(0.4);
        sleep(sleep);
    }

    private void strafeleft(long sleep) {
        frontleftmotor.setPower(-0.4);
        backleftmotor.setPower(0.4);
        frontrightmotor.setPower(0.4);
        backrightmotor.setPower(-0.4);
        sleep(sleep);
    }
    
    // private void dropmarker(); {
    //     teammarker.setPosition(1.0);
    //     sleep(100);
    //     teammarker.setPosition(0.0);
    // }
    
    private void stopMotor() {
        frontleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    /*private moveuntildistance(){
        while (sensorRange.getDistance(DistanceUnit.INCH>36){
            frontrightmotor.setPower(-frontrightpower);
            frontleftmotor.setPower(frontleftpower);
            backrightmotor.setPower(-backrightpower);
            backleftmotor.setPower(backrightpower);
        }
    }
    */
    private void DistanceFormula(double x1, double y1, double x2, double y2, char direction) {
        double final_x = Math.pow(x2 - x1, 2);
        double final_y = Math.pow(y2 - y1, 2);
        double distance = (Math.sqrt(final_x + final_y)) * 60.96;
        System.out.println(distance);
        //telemtry.addLine(distance);
        if (direction == 'f' || direction == 'b') {
            double first_step = 11;
            double second_step = 8.5;
            double next_steps = 6;
            double sleep = Math.rint(((((distance-first_step)-second_step)/next_steps)*100)+200);
            long sleep2 = Math.round(sleep);
            forward_backward(direction, sleep2);
        } else if (direction == 'l' || direction == 'r') {
            double first_strafe = 3;
            double next_strafe = 3;
            double sleep = Math.rint(((distance-first_strafe)/next_strafe*100)+100);
            long sleep2 = Math.round(sleep);
            strafedistance(direction,  sleep2);
        }
    }

    private void forward_backward(char direction, long sleep) {
        if (direction == 'f') {
            moveForward(sleep);
            stopMotor();
        }

        if (direction == 'b') {
            moveBackward(sleep);
            stopMotor();
        }
    }

    // private void turn(int degrees) {
    //     double new_degrees = Math.rint(8.05 * degrees);// degrees was angleDeg
    //     double turn= new_degrees - new_degrees; /*the second "new_degrees" needs to be replaced with the amount the robot has turned since this while loop started*/
    //     while (turn > 2) {//
    //     turn = new_degrees - new_degrees; /*the second "new_degrees" needs to be replaced with the amount the robot has turned since this while loop started*/
    //         if (new_degrees > 0) {
    //             spinright();
    //         }
    //         else if (degrees < 0) {
    //             spinleft();
    //         }
    //         else{
    //             stopMotor();
    //         }
    //         }


    //     stopMotor();

    // }
    
    private void turn1(int degrees, char side) {
        float secperdegree = (float)10;
        int turn = (int)(secperdegree * degrees);
        
        if (side == 'l') {
            spinleft(turn);
            
        }
        
        if (side == 'r') {
            spinright(turn);
            
        }
    }

    private void strafedistance(char direction, long sleep) {
        if (direction == 'r') {
            straferight(sleep);
            stopMotor();
        }

        if (direction == 'l') {
            strafeleft(sleep);
            stopMotor();
        }
    }
    
    private void unhang() {
        hanging1.setPower(0.7);
        hanging2.setPower(0.7);
        sleep(5200);
        hanging1.setPower(0.0);
        hanging2.setPower(0.0);
        
    }
    private void unhook() {
        //unhang();
        DistanceFormula(0.75, 0.5, 0.6, 0.8, 'l');
        sleep(100);
        DistanceFormula(0.5, 0.9, 0.8, 1.2, 'f');
        sleep(100);
        DistanceFormula(0.8, 1.2, 1, 1, 'r');
    }
    
    private void centerpath() {
        DistanceFormula(0.6, -0.8, 1.2, -1.2, 'f');
        //sleep(100);
        DistanceFormula(1.2, -1.2, 0.9, -0.9, 'b');
        sleep(100);
        turn1(90, 'l');
        sleep(100);
        DistanceFormula(0.8, -0.8, 2.5, 0.5, 'f');
        sleep(100);
        turn1(55, 'l');
        sleep(100);
        DistanceFormula(2.5, 1, 2.3, 2.75, 'f');
        sleep(100);
        //dropmarker();
        sleep(100);
        //DistanceFormula(2.5, 2.75, 3, 3, 'r');
        // sleep(1000);
        //moveuntildistance();
        //stopMotor();
        moveBackward(6000);
    }
    
    private void leftpath() {
        DistanceFormula(0.6, -0.8, 0.8, -0.8, 'f');
        sleep(100);
        DistanceFormula(0.8, -0.8, 1.5, -0.75, 'l');
        sleep(100);
        DistanceFormula(1.5, -0.75, 2, -1, 'f');
        sleep(100);
        DistanceFormula(2, -1, 1.5, -0.75, 'b');
        sleep(100);
        turn1(90, 'l');
        sleep(100);
        DistanceFormula(1.75, -0.35, 2.5, 1, 'f');
        sleep(100);
        turn1(55, 'l');
        DistanceFormula(2.5, 1.2, 2.5, 2.75, 'f');
        sleep(100);
        //DistanceFormula(2.5, 2.75, 2.75, 2.75, 'r');
        //turn1(90, 'r');
        sleep(1000);
        //dropmarker();
        //moveuntildistance();
        //stopMotor();
        moveBackward(6000);
    }
    
    private void rightpath() {
        DistanceFormula(0.6, -0.8, 0.8, -0.8, 'f');
        sleep(100);
        DistanceFormula(0.8, -0.8, 0.2, -1.5, 'r');
        sleep(100);
        DistanceFormula(0.5, -1.5, 1, -2, 'f');
        sleep(100);
        DistanceFormula(1, -2, 0.5, -1.2, 'b');
        sleep(100);
        turn1(90, 'l');
        sleep(100);
        DistanceFormula(0.5, -1.2, 2.5, 0.2, 'f');
        turn1(45, 'l');
        sleep(100);
        DistanceFormula(2.5, 0.2, 2.5, 2.75, 'f');
        sleep(100);
        DistanceFormula(2.5, 2.75, 2.75, 2.75, 'r');
        sleep(100);
        // //dropmarker();
        // //moveuntildistance();
        // //stopMotor();
        moveBackward(6000);
    }

    /*private void autonomous() {
        //moveForward();
        strafeleft();
        sleep(200);
        stop();
    }*/
}
