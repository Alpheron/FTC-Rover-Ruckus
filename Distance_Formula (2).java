package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "DistanceFormula", group = "Concept")

public class Distance_Formula extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final int LEFT =0;
    private static final int CENTER =1;
    private static final int RIGHT =2;
    BNO055IMU imu;
 private static final String VUFORIA_KEY = "ATWusJz/////AAABmYzqHQl+0kl4p5WlsUb9jL2BkOURyN7eotmsvaYqZ6vfFe2fO9YSQMQaKp8/5z4EOFjr0ZVXCYhJO95G3zsPbrVfcAgCxPtDQ7QiGY+ykVMCw5UuI/nQ4Zh7+dvKIT4CFwNnG4KaWnYXRT9NW2J7noRWQ/vLcEC5tXjlY2inAy7eC0BanVf5aMX/SMl1qm+pWu6TonjVDQlcuBIwGlFWI6nJzNkCyDyQIqgbLrSMMn9kvQgj3lUyxK3HxqqX/XW2Xup/mr7fwQxj6gvTFUlj21CDlx1r9RqizEuUDrN5UwuguNFyc7qbRo44RM98L/Ab81kXVCmfG9JPCGZ5c2mfQ+z0YTjRscHTlWRMBsom83RT";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;



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
    private int position = -1;



    @Override
    public void runOpMode() {
        initVuforia();
        
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        
        frontleftmotor = hardwareMap.get(DcMotor.class, "frontleftmotor");
        frontrightmotor = hardwareMap.get(DcMotor.class, "frontrightmotor");
        backrightmotor = hardwareMap.get(DcMotor.class, "backrightmotor");
        backleftmotor = hardwareMap.get(DcMotor.class, "backleftmotor");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
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

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        //imu.initialize(parameters);
        teammarker.setPosition(1.0);
        waitForStart();
        
        if (opModeIsActive()) {
          
          if (tfod != null) {
            tfod.activate();
            
        }
        //unhang(); 
        sleep(1000);
        while (opModeIsActive()){
          readPosition();
          
          if(position!=-1){
            break;
            
          }
          }
        }
        //composeTelemetry();
        //unhook();
        unhook();
        moveToMineral();
        
        if (tfod != null) {
            tfod.shutdown();
        }
    
      
    }
    
    private void checkDeviceCompatibility(){
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
  }
  
        //leftpath();
        
        
     private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine
    }
    private void readPosition(){
        
      if (tfod != null) {
         //position = LEFT;
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected by BnB is ", updatedRecognitions.size());
                      if (updatedRecognitions.size() == 2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                          //telemetry.addData("Value of object place:", recognition.getLeft());
                          if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getRight();
                            telemetry.addData("Value of object place:", recognition.getLeft());
                          } else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                              if (silverMineral1X == -1){
                            silverMineral1X = (int) recognition.getLeft();
                              }
                              else {
                                  silverMineral2X = (int) recognition.getLeft();
                              }
                          }
                          
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1) {
                          
                          telemetry.addData("Gold Mineral goldMineralX ",goldMineralX);
                          telemetry.addData("Gold Mineral silverMineral1X ", silverMineral1X);
                         // telemetry.addData("Gold Mineral silverMineral2X ",silverMineral2X );
                          
                          
                          if (goldMineralX < silverMineral1X) {
                            telemetry.addData("Gold Mineral Position", "Center");
                            position = CENTER;
                            
                          } else {
                            telemetry.addData("Gold Mineral Position", "Right");
                            position = RIGHT;
                          }
                        }
                        else if(silverMineral2X != -1 && silverMineral1X != -1) {
                          
                          //telemetry.addData("Gold Mineral goldMineralX ",goldMineralX);
                          telemetry.addData("Gold Mineral silverMineral1X ", silverMineral1X);
                          telemetry.addData("Gold Mineral silverMineral2X ",silverMineral2X );
                  
                            telemetry.addData("Gold Mineral Position", "Left");
                            position = LEFT;
                          }
                        }
                      telemetry.update();
                    }
                }
    }
        
        
    private void moveToMineral(){
      if (position == LEFT){
          telemetry.addLine("Path Left");
        leftpath();
      }
      else if (position == CENTER){
         telemetry.addLine("Path Center");
        centerpath();
      }else if (position == RIGHT){
          telemetry.addLine("Path Right");
        rightpath();
      }else{
        centerpath();
        
      }
      }   
      
        
      
    



    private void moveForward(long sleep){
        frontrightmotor.setPower(frontrightpower);
        frontleftmotor.setPower(-frontleftpower);
        backrightmotor.setPower(backrightpower);
        backleftmotor.setPower(-backleftpower);
        sleep(sleep);
        stopMotor();

    }

    private void moveBackward(long sleep){
        frontrightmotor.setPower(-frontrightpower);
        frontleftmotor.setPower(frontleftpower);
        backrightmotor.setPower(-backrightpower);
        backleftmotor.setPower(backrightpower);
        sleep(sleep);
        stopMotor();
    }


    private void spinleft(long sleep){
        frontrightmotor.setPower(frontrightpower);
        frontleftmotor.setPower(frontleftpower);
        backrightmotor.setPower(backrightpower);
        backleftmotor.setPower(backleftpower);
        sleep(sleep);
        stopMotor();
        
        

    }

    private void spinright(long sleep){
        frontrightmotor.setPower(-frontrightpower);
        frontleftmotor.setPower(-frontleftpower);
        backrightmotor.setPower(-backrightpower);
        backleftmotor.setPower(-backleftpower);
        sleep(sleep);
        stopMotor();
        
    }

    private void straferight(long sleep) {
        frontleftmotor.setPower(-0.4);
        backleftmotor.setPower(0.4);
        frontrightmotor.setPower(-0.4);
        backrightmotor.setPower(0.4);
        sleep(sleep);
        stopMotor();
    }

    private void strafeleft(long sleep) {
        frontleftmotor.setPower(0.4);
        backleftmotor.setPower(-0.4);
        frontrightmotor.setPower(0.4);
        backrightmotor.setPower(-0.4);
        sleep(sleep);
        stopMotor();
    }
    
    private void dropmarker() {
         teammarker.setPosition(0.75);
         sleep(1000);
         teammarker.setPosition(0.0);
    }
    
    /*private void moveuntildistance() {
        while (sensorRange.getDistance(DistanceUnit.INCH>36)){
            frontrightmotor.setPower(-frontrightpower);
            frontleftmotor.setPower(frontleftpower);
            backrightmotor.setPower(-backrightpower);
            backleftmotor.setPower(backrightpower);
        }
    }
    */
    private void stopMotor() {
        
        frontleftmotor.setPower(0.0);
        backleftmotor.setPower(0.0);
        frontrightmotor.setPower(0.0);
        backrightmotor.setPower(0.0);
        frontleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }    
   
    private void DistanceFormula(double x1, double y1, double x2, double y2, char direction) {
        double final_x = Math.pow(x2 - x1, 2);
        double final_y = Math.pow(y2 - y1, 2);
        double distance = (Math.sqrt(final_x + final_y)) * 60.96;
        System.out.println(distance);
        //telemtry.addLine(distance);
        if (direction == 'f' || direction == 'b') {
            double first_step = 11.0;
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
        hanging1.setPower(-0.7);
        hanging2.setPower(-0.7);
        sleep(4000);
        hanging1.setPower(0.0);
        hanging2.setPower(0.0);
        sleep(1000);
        
    }
    private void unhook() {
       // unhang();
       //stopMotor();
        //sleep(400);
        
        DistanceFormula(0.75, 0.5, 0.6, 0.8, 'l');
        sleep(100);
        DistanceFormula(0.5, 0.9, 0.8, 1.2, 'f');
        sleep(100);
        DistanceFormula(0.8, 1.2, 1.1, 1, 'r');
        sleep(100);
        stopMotor();
        turn1(5, 'r');
        //readPosition();
    }
    
    private void headBack(){
        DistanceFormula(3, 2.75, 2, 3, 'b');
       // DistanceFormula(2, 3, 2, 2.9, 'l');
        DistanceFormula(2, 1.5, 0, 2.5, 'b');
    }
    
    private void failsafe() {
        DistanceFormula(0.5, 1.5, -0.25, 2.5, 'l');
        turn1(120, 'r');
        DistanceFormula(-0.25, 2.5, 2.5, 2.5, 'f');
        dropmarker();
        headBack();
        
        
    }
    
    private void centerpath() {
        DistanceFormula(1, 1, 2.5, 2.5, 'f');
        sleep(100);
        turn1(60, 'r');
        // sleep(100);
        DistanceFormula(2.5, 2.5, 2.75, 2.75, 'f');
        // sleep(100);
        // //DistanceFormula(2.3, 2.3, 2.2, 2.2, 'b');
       
        //  sleep(600);
        
        DistanceFormula(2.75, 2.75, 3.2, 3.2, 'l');
          //turn1(20, 'l');
        // // sleep(100);
        
        dropmarker();
        
        // // sleep(1000);
        // // //moveuntildistance();
        // // //stopMotor();
        headBack();
        
    }
    
    private void leftpath() {
        DistanceFormula(1, 1, 1.3, 1.3, 'f');
        sleep(100);
        DistanceFormula(1.2, 1.2, 0.75, 1.9, 'l');
        sleep(100);
        //turn1(17, 'r');
        DistanceFormula(0.9, 1.5, 2, 2.5, 'f');
        sleep(100);
        turn1(63, 'r');
        sleep(100);
        DistanceFormula(1.5, 2.3, 1.5, 2.5, 'l');
        sleep(100);
        DistanceFormula(1.5, 2.5, 2.75, 2.75, 'f');
        sleep(100);
        DistanceFormula(2.5, 2.25, 2.5, 2.5, 'l');
        sleep(100);
        dropmarker();
        // sleep(1000);
       
        // //moveuntildistance();
        // //stopMotor();
        //DistanceFormula(2.5, 2.5, -0.25, 3, 'b');
        headBack();
        
    }
    
    private void rightpath() {
        DistanceFormula(1, 1, 1.3, 1.3, 'f');
        sleep(100);
        DistanceFormula(1.2, 1.2, 1.5, 0.425, 'r');
        sleep(100);
        DistanceFormula(1.5, 0.425, 2.5, 2, 'f');
        sleep(100);
        turn1(45, 'l');
        sleep(100);
        DistanceFormula(2.5, 2, 2.8, 2.8, 'f');
        sleep(100);
        turn1(105, 'r');
        DistanceFormula(2.75, 2.75, 2.75, 2.7, 'l');
        DistanceFormula(2.75, 2.7, 2.75, 2.9, 'l');
        dropmarker();
        sleep(1000);
        // //moveuntildistance();
        // //stopMotor();
        headBack();
        
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();
        //Use above code in game to save battery
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        //tfodParameters.useObjectTracker = true;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
