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


@Autonomous(name = "Distance_Formula", group = "Concept")

public class Distance_Formula extends LinearOpMode {
    BNO055IMU imu;



    private ElapsedTime runtime = new ElapsedTime();


    private static DcMotor frontleftmotor = null;
    private static DcMotor frontrightmotor = null;
    private static DcMotor backleftmotor = null;
    private static DcMotor backrightmotor = null;
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
        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        //imu.initialize(parameters);

        waitForStart();
        //composeTelemetry();

        
        DistanceFormula(0.75, 0.5, 0.5, 0.9, 'l');

        DistanceFormula(0.5, 0.9, 0.8, 1.2, 'f');
      
        DistanceFormula(0.8, 1.2, 1, 1, 'r');
      
        DistanceFormula(1, 1, 2, 2.5, 'f');
      
        turn1(45);
        
        DistanceFormula(2, 2, 3, 2, 'f');
       
        DistanceFormula(3, 2, 3, 2.75, 'l');
       
        sleep(1000);
        
        DistanceFormula(3, 2.75, -1, 3, 'b');
      
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
        frontleftmotor.setPower(-frontleftpower);
        backleftmotor.setPower(backleftpower);
        frontrightmotor.setPower(-frontrightpower);
        backrightmotor.setPower(backrightpower);
        sleep(sleep);
    }

    private void strafeleft(long sleep) {
        frontleftmotor.setPower(frontleftpower);
        backleftmotor.setPower(-backleftpower);
        frontrightmotor.setPower(frontrightpower);
        backrightmotor.setPower(-backrightpower);
        sleep(sleep);
    }

    private void stopMotor() {
        frontleftmotor.setPower(0.0);
        backleftmotor.setPower(0.0);
        frontrightmotor.setPower(0.0);
        backrightmotor.setPower(0.0);
    }
    private void DistanceFormula(double x1, double y1, double x2, double y2, char direction) {
        double final_x = Math.pow(x2 - x1, 2);
        double final_y = Math.pow(y2 - y1, 2);
        double distance = (Math.sqrt(final_x + final_y)) * 60.96;
        System.out.println(distance);
        //telemtry.addLine(distance);
        if (direction == 'f' || direction == 'b') {
            int first_step = 13;
            int next_steps = 8;
            double sleep = Math.rint((((distance-first_step)/next_steps)*100)+100);
            long sleep2 = Math.round(sleep);
            forward_backward(direction, sleep2);
        } else if (direction == 'l' || direction == 'r') {
            double sleep = Math.rint(distance*33.3333333333);
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
    
    private void turn1(int degrees) {
        float millisecperdegree = (float)8.04;
        int turn = (int)(millisecperdegree * degrees);
        
        if (degrees < 0) {
            spinleft(turn);
            
        }
        
        if (degrees > 0) {
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

    /*private void autonomous() {
        //moveForward();
        strafeleft();
        sleep(200);
        stop();
    }*/
}
