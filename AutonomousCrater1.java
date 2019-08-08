package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name="AutonomousCrater1", group="Concept")
//@Disabled
public class AutonomousCrater1 extends LinearOpMode
{
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final int LEFT =0;
    private static final int CENTER =1;
    private static final int RIGHT =2;
    DcMotor                 frontleftmotor;
    DcMotor                 frontrightmotor;
    DcMotor                 backleftmotor;
    DcMotor                 backrightmotor;
    DcMotor                 hanging1;
    DcMotor                 hanging2;
    Servo                   output;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .675, correction;
    PIDController           pidRotate, pidDrive;
    private static final String VUFORIA_KEY = "ATWusJz/////AAABmYzqHQl+0kl4p5WlsUb9jL2BkOURyN7eotmsvaYqZ6vfFe2fO9YSQMQaKp8/5z4EOFjr0ZVXCYhJO95G3zsPbrVfcAgCxPtDQ7QiGY+ykVMCw5UuI/nQ4Zh7+dvKIT4CFwNnG4KaWnYXRT9NW2J7noRWQ/vLcEC5tXjlY2inAy7eC0BanVf5aMX/SMl1qm+pWu6TonjVDQlcuBIwGlFWI6nJzNkCyDyQIqgbLrSMMn9kvQgj3lUyxK3HxqqX/XW2Xup/mr7fwQxj6gvTFUlj21CDlx1r9RqizEuUDrN5UwuguNFyc7qbRo44RM98L/Ab81kXVCmfG9JPCGZ5c2mfQ+z0YTjRscHTlWRMBsom83RT";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private int position = -1;
    
    @Override
    public void runOpMode() 
    {
        initVuforia();
         if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        initDrive();
        initPID_Gyro();
        init_PID();
        
        waitForStart();
        if (opModeIsActive()) {
          
         if (tfod != null) {
            tfod.activate();
            
        }
        //alignRobot(200);
        while (opModeIsActive()){
          readPosition();
          
          if(position!=-1){
            break;
            
          }
        }
        //composeTelemetry();
       
        readPosition();
        moveToMineral();
        
        if (tfod != null) {
            tfod.shutdown();
        }

        }   
        
}
  
        
        
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
    private void moveToMineral(){
      if (position == LEFT){
        leftPath();
      }
      else if (position == CENTER){
        centerPath();
      }else if (position == RIGHT){
        rightPath();
      }else{
        defaultPath();
      }
      }
    
    private void DistanceFormula(double x1, double y1, double x2, double y2, char direction, char special) {
     //*******************************************************
      //Calculating Distance from points A = (x2, y2) & B = (x2, y2) & C = (x3, y3)
      //Used distance formula
      double length_ab = Math.sqrt(Math.pow((x2 - x1), 2) + Math.pow((y2 - y1), 2));
      double distance = length_ab*24;
      double length_bc = Math.sqrt(Math.pow((x2 - x2), 2) + Math.pow((y2 - y1), 2));
      //*******************************************************
      double angle_b = Math.rint(Math.toDegrees(Math.acos(length_bc/length_ab))); //adj/hyp
      //*********************************************************
      int turn_angle = (int)0.0;
      if (x1 <= x2) {
        if (y1 <= y2) {
          turn_angle = (int)Math.round(-(angle_b));
        }

        else if (y1 > y2) {
          turn_angle = (int)Math.round(-(180-angle_b));
        }
         
      }

      else if (x1 > x2) {
        if (y1 < y2) {
          turn_angle = (int)Math.round(-angle_b);
        }

        else if (y1 >= y2) {
          turn_angle = (int)Math.round((angle_b -180));
        }
      }
      telemetry.addData("The distance is",distance);
      telemetry.addData("The angle to be turned is", turn_angle+45);
      telemetry.update();
      
      if (direction == 'f' || direction == 'b'){
          double circumference = (4*Math.PI);  
          double rotation = (distance/circumference);
        //   telemetry.addData("The number of rotations are", rotation);
          int target1 = (int)Math.round(rotation * 1120);
          //telemetry.addData("the # of ticks are", target1);
          //telemetry.update();
          forward_backward(direction, special, turn_angle+135, target1);
          }
      else if (direction == 'l' || direction == 'r') {
          double strafeDist = 8; //1 rotation in inches
          int target2 = (int)Math.round((distance/strafeDist)*1120);
          strafeDirection(direction, target2, special);
      }
  }

    private void moveForward(int target){
        int currentpos = (int)(Math.rint((frontrightmotor.getCurrentPosition() + frontleftmotor.getCurrentPosition() + backrightmotor.getCurrentPosition() + backleftmotor.getCurrentPosition())/4));
        int TOLERANCE = 200;
        while (opModeIsActive() && (Math.abs(currentpos-target)>=TOLERANCE)) //something is wrong with this while loop, it keeps going forever
        {
            currentpos = (int)(Math.rint((Math.abs(frontrightmotor.getCurrentPosition()) + Math.abs(frontleftmotor.getCurrentPosition()) + Math.abs(backrightmotor.getCurrentPosition()) + Math.abs(backleftmotor.getCurrentPosition()))/4));

            //Display info to driver
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.addData("The distance is:",target);
            telemetry.addData("The current position is:",currentpos);
            telemetry.addData("The conditional is:", currentpos-target);
            
            telemetry.update();

            // Use PID with imu input to drive in a straight line.
            correction = pidDrive.performPID(getAngle());

            frontleftmotor.setPower(-power + correction);
            backleftmotor.setPower(-power + correction);
            frontrightmotor.setPower(-power);
            backrightmotor.setPower(-power);
            
            if (currentpos>=target-20) {
                break;
            }
            }

        // turn the motors off.
        stopMotor();
    
}
    
    private void moveBackward(int target){
        int currentpos = (int)(Math.rint((Math.abs(frontrightmotor.getCurrentPosition()) + Math.abs(frontleftmotor.getCurrentPosition()) + Math.abs(backrightmotor.getCurrentPosition()) + Math.abs(backleftmotor.getCurrentPosition()))/4));
        int TOLERANCE = 200;
        while (opModeIsActive() && (Math.abs(currentpos-target)>TOLERANCE)) //something is wrong with this while loop, it keeps going forever
        {
            currentpos = (int)(Math.rint((frontrightmotor.getCurrentPosition() + frontleftmotor.getCurrentPosition() + backrightmotor.getCurrentPosition() + backleftmotor.getCurrentPosition())/4));

            //Display info to driver
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.addData("The distance is:",target);
            telemetry.addData("The current position is:",currentpos);
            telemetry.addData("The conditional is:", currentpos-target);
            
            telemetry.update();

            // Use PID with imu input to drive in a straight line.
            correction = pidDrive.performPID(getAngle());

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();

            // set power levels.
            frontleftmotor.setPower(power );
            backleftmotor.setPower(power);
            frontrightmotor.setPower(power - correction);
            backrightmotor.setPower(power - correction);
            
            if (Math.abs(currentpos)>=target-200) {
                break;
            }
    }
    stopMotor();
    }
    
    private void hail_mary(){
        frontleftmotor.setPower(1.0);
        backleftmotor.setPower(1.0);
        frontrightmotor.setPower(1.0);
        backrightmotor.setPower(1.0);
        sleep(4000);
        stopMotor();
    }
    
    private void forward_backward(char direction, char special, int angle, int target) {
        if (direction == 'f') {
            if (special == '*'){
            rotate(angle, power);
            moveForward(target);
            stopMotor();
            
            }
            
            else if (special == 'n') {
                moveForward(target);
                stopMotor();
                
            }
        }
        if (direction == 'b') {
            if (special == '*'){
            rotate(angle, power);
            moveBackward(target);
            stopMotor();
            
            }
            
            else if (special == 'n'){
                moveBackward(target);
                stopMotor();
                
            }
        }
        }
    
    private void moveLeft(int distance){
        frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        frontleftmotor.setTargetPosition(distance);
        frontrightmotor.setTargetPosition(-distance);
        backleftmotor.setTargetPosition(-distance);
        backrightmotor.setTargetPosition(distance);

        frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        frontleftmotor.setPower(power);
        frontrightmotor.setPower(-power);
        backleftmotor.setPower(-power);
        backrightmotor.setPower(power);
        telemetry.addData("time: Right", frontleftmotor.getCurrentPosition());
        while(opModeIsActive() && (-frontleftmotor.getCurrentPosition())< distance){
            if (backrightmotor.getCurrentPosition() >= distance-20){ 
                break;
        }
          
        }
        stopMotor();
        
    }
    
    private void moveRight(int distance) {
        frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        frontleftmotor.setTargetPosition(-distance);
        frontrightmotor.setTargetPosition(distance);
        backleftmotor.setTargetPosition(distance);
        backrightmotor.setTargetPosition(-distance);

        frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        frontleftmotor.setPower(-power);
        frontrightmotor.setPower(power);
        backleftmotor.setPower(power);
        backrightmotor.setPower(power);
        telemetry.addData("time: Left", frontleftmotor.getCurrentPosition());
        telemetry.addLine("");
        telemetry.update();
        while(opModeIsActive() && (frontleftmotor.getCurrentPosition())< distance){
            if (backrightmotor.getCurrentPosition() >= distance-20){ 
                break;
        }
        }
        stopMotor();
        
}
        
    private void TurnFromCurrentPos(){
        int angle = (int)(0-getAngle());
        telemetry.addData("The current angle is", getAngle());
        telemetry.addData("The angle to be turned is", angle);
        telemetry.update();
        rotate(angle, power);
        
            
    }
    
    private void strafeDirection(char direction, int distance, int special) {
        if (direction == 'l') {

           if (special == '*'){
            moveLeft(distance);
            TurnFromCurrentPos();
            stopMotor();
            
            }
            
            else if (special == 's') {
                moveLeft(distance);
                stopMotor();
        }
        }
        
        if (direction == 'r') {
           if (special == '*'){
            moveRight(distance);
            TurnFromCurrentPos();
            stopMotor();
            
            }
            
            else if (special == 's') {
                moveRight(distance);
                stopMotor();
          
        }
    }
    }
    private void stopMotor()
    {
        frontleftmotor.setPower(0.0);
        backleftmotor.setPower(0.0);
        frontrightmotor.setPower(0.0);
        backrightmotor.setPower(0.0);
        hanging1.setPower(0.0);
        hanging2.setPower(0.0);
        
        frontleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        hanging1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hanging2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hanging1.setDirection(DcMotor.Direction.REVERSE);
        
        frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //sleep(500);
        frontleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        //resetAngle();
    }
    
    private void unhang() {
        hanging1.setPower(-0.7);
        hanging2.setPower(-0.7);
        sleep(4950);
        stopMotor();
        TurnFromCurrentPos();
        
    }
    
     private void unhook() {
        //unhang();
        DistanceFormula(0.75, 0.75, 0.5, 1, 'r', '*');
        DistanceFormula(0.5, 1, 0.8, 1.2, 'f', 'n');
        DistanceFormula(0.8, 1.2, 0.5, 1, 'l','*');
        telemetry.addLine("this works");
        telemetry.update();
        
     }
     
    private void defaultPath(){
        unhook();
        DistanceFormula(0.6, -0.8, 1.1, -1.1, 'f', 'n');
        rotate(45, power);
        DistanceFormula(1.1, -1.1, 2.6, 0.5, 'f', 'n');
        rotate(45, power);
        DistanceFormula(2.6, 0.5, 2.5, 2.7, 'f', 'n');
        rotate(-90, power);
        dropmarker();
        sleep(700);
        rotate(90, power);
        hail_mary();
    }
     
    private void centerPath(){
        unhook();
        DistanceFormula(0.6, -0.8, 1.3, -1.3, 'f', 'n');
        DistanceFormula(1.3, -1.3, 1.1, -1.1, 'b', 'n');
        rotate(90, power);
        DistanceFormula(0.8, -0.8, 2.75, 0.5, 'f', 'n');
        rotate(40, power);
        DistanceFormula(2.5, 0.5, 2.5, 2, 'f', 'n');
        rotate(-90, power);
        dropmarker();
        sleep(700);
        DistanceFormula(2.5, 2.5, 2.75, 2.5, 'f', 'n');
        
        rotate(82, power);
        hail_mary();
        
    }
    
    private void rightPath(){
        unhook();
        DistanceFormula(0.6, -0.8, 0.8, -0.8, 'f', 'n');
        DistanceFormula(1, -1, 0.75, -1.8, 'r', '*');
        DistanceFormula(0.75, -1.8, 1, -2, 'f', 'n');
        DistanceFormula(1, -2.2, 0.8, -1.8, 'b', 'n');
        rotate(90, power);
        DistanceFormula(0.8, -1.8, 2, 0.5, 'f', 'n');
        rotate(45, power);
        DistanceFormula(2.5, 0.5, 2.5, 2.25, 'f', 'n');
        rotate(-90, power);
        dropmarker();
        sleep(200);
        rotate(90, power);
        //DistanceFormula
        hail_mary();
    }
    
    private void leftPath(){
        unhook();
        DistanceFormula(0.6, -0.8, 0.8, -0.8, 'f', 'n');
        DistanceFormula(0.8, -0.8, 1.5, -0.9, 'l', '*');
        DistanceFormula(1.5, -0.75, 2.3, -1, 'f', 'n');
        DistanceFormula(2, -1, 1.75, -0.75, 'b', 'n');
        rotate(90, power);
        DistanceFormula(1.75, -0.75, 1.85, 0.4, 'f', 'n');
        rotate(35, power);
        DistanceFormula(2.7, 0.4, 2.7, 2.3, 'f', 'n');
        rotate(-90, power);
        dropmarker();
        sleep(400);
        rotate(90, power);
        hail_mary();
    }
    
     private void dropmarker() {
         output.setPosition(1.0);
         sleep(100);
         output.setPosition(0.3);
    }
    
    private void initDrive(){
        frontleftmotor = hardwareMap.get(DcMotor.class, "frontleftmotor");
        frontrightmotor = hardwareMap.get(DcMotor.class, "frontrightmotor");
        backrightmotor = hardwareMap.get(DcMotor.class, "backrightmotor");
        backleftmotor = hardwareMap.get(DcMotor.class, "backleftmotor");
        backrightmotor.setDirection(DcMotor.Direction.REVERSE);
        frontrightmotor.setDirection(DcMotor.Direction.REVERSE);
        hanging1 = hardwareMap.get(DcMotor.class, "hanging1");
        hanging2 = hardwareMap.get(DcMotor.class, "hanging2");
        output = hardwareMap.get(Servo.class, "output");
        stopMotor();
    }
    
    private void initPID_Gyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        pidRotate = new PIDController(.005, 0, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        
        telemetry.update();

        // wait for start button.

    }
    
    private void init_PID(){
        telemetry.addData("OpMode", "Started");
        telemetry.update();
        sleep(1000);
        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();
    }
    
    
    
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        // restart imu angle tracking.
        resetAngle();

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle with a minimum of 20%.
        // This is to prevent the robots momentum from overshooting the turn after we turn off the
        // power. The PID controller reports onTarget() = true when the difference between turn
        // angle and target angle is within 2% of target (tolerance). This helps prevent overshoot.
        // The minimum power is determined by testing and must enough to prevent motor stall and
        // complete the turn. Note: if the gap between the starting power and the stall (minimum)
        // power is small, overshoot may still occur. Overshoot is dependant on the motor and
        // gearing configuration, starting power, weight of the robot and the on target tolerance.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, 90);
        pidRotate.setOutputRange(.50, power);
        pidRotate.setTolerance(2);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                frontleftmotor.setPower(-power);
                backleftmotor.setPower(-power);
                frontrightmotor.setPower(power);
                backrightmotor.setPower(power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                frontleftmotor.setPower(power);
                backleftmotor.setPower(power);
                frontrightmotor.setPower(-power);
                backrightmotor.setPower(-power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                frontleftmotor.setPower(power);
                backleftmotor.setPower(power);
                frontrightmotor.setPower(-power);
                backrightmotor.setPower(-power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        stopMotor();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

    // PID controller courtesy of Peter Tischler, with modifications.

    public class PIDController
    {
        private double m_P;                                 // factor for "proportional" control
        private double m_I;                                 // factor for "integral" control
        private double m_D;                                 // factor for "derivative" control
        private double m_input;                 // sensor input for pid controller
        private double m_maximumOutput = 1.0;   // |maximum output|
        private double m_minimumOutput = -1.0;  // |minimum output|
        private double m_maximumInput = 0.0;    // maximum input - limit setpoint to this
        private double m_minimumInput = 0.0;    // minimum input - limit setpoint to this
        private boolean m_continuous = false;   // do the endpoints wrap around? eg. Absolute encoder
        private boolean m_enabled = false;              //is the pid controller enabled
        private double m_prevError = 0.0;           // the prior sensor input (used to compute velocity)
        private double m_totalError = 0.0;      //the sum of the errors for use in the integral calc
        private double m_tolerance = 0.05;          //the percentage error that is considered on target
        private double m_setpoint = 0.0;
        private double m_error = 0.0;
        private double m_result = 0.0;

        /**
         * Allocate a PID object with the given constants for P, I, D
         * @param Kp the proportional coefficient
         * @param Ki the integral coefficient
         * @param Kd the derivative coefficient
         */
        public PIDController(double Kp, double Ki, double Kd)
        {
            m_P = Kp;
            m_I = Ki;
            m_D = Kd;
        }

        /**
         * Read the input, calculate the output accordingly, and write to the output.
         * This should only be called by the PIDTask
         * and is created during initialization.
         */
        private void calculate()
        {
            int     sign = 1;

            // If enabled then proceed into controller calculations
            if (m_enabled)
            {
                // Calculate the error signal
                m_error = m_setpoint - m_input;

                // If continuous is set to true allow wrap around
                if (m_continuous)
                {
                    if (Math.abs(m_error) > (m_maximumInput - m_minimumInput) / 2)
                    {
                        if (m_error > 0)
                            m_error = m_error - m_maximumInput + m_minimumInput;
                        else
                            m_error = m_error + m_maximumInput - m_minimumInput;
                    }
                }

                // Integrate the errors as long as the upcoming integrator does
                // not exceed the minimum and maximum output thresholds.

                if ((Math.abs(m_totalError + m_error) * m_I < m_maximumOutput) &&
                        (Math.abs(m_totalError + m_error) * m_I > m_minimumOutput))
                    m_totalError += m_error;

                // Perform the primary PID calculation
                m_result = m_P * m_error + m_I * m_totalError + m_D * (m_error - m_prevError);

                // Set the current error to the previous error for the next cycle.
                m_prevError = m_error;

                if (m_result < 0) sign = -1;    // Record sign of result.

                // Make sure the final result is within bounds. If we constrain the result, we make
                // sure the sign of the constrained result matches the original result sign.
                if (Math.abs(m_result) > m_maximumOutput)
                    m_result = m_maximumOutput * sign;
                else if (Math.abs(m_result) < m_minimumOutput)
                    m_result = m_minimumOutput * sign;
            }
        }

        /**
         * Set the PID Controller gain parameters.
         * Set the proportional, integral, and differential coefficients.
         * @param p Proportional coefficient
         * @param i Integral coefficient
         * @param d Differential coefficient
         */
        public void setPID(double p, double i, double d)
        {
            m_P = p;
            m_I = i;
            m_D = d;
        }

        /**
         * Get the Proportional coefficient
         * @return proportional coefficient
         */
        public double getP() {
            return m_P;
        }

        /**
         * Get the Integral coefficient
         * @return integral coefficient
         */
        public double getI() {
            return m_I;
        }

        /**
         * Get the Differential coefficient
         * @return differential coefficient
         */
        public double getD() {
            return m_D;
        }

        /**
         * Return the current PID result for the last input set with setInput().
         * This is always centered on zero and constrained the the max and min outs
         * @return the latest calculated output
         */
        public double performPID()
        {
            calculate();
            return m_result;
        }

        /**
         * Return the current PID result for the specified input.
         * @param input The input value to be used to calculate the PID result.
         * This is always centered on zero and constrained the the max and min outs
         * @return the latest calculated output
         */
        public double performPID(double input)
        {
            setInput(input);
            return performPID();
        }

        /**
         *  Set the PID controller to consider the input to be continuous,
         *  Rather then using the max and min in as constraints, it considers them to
         *  be the same point and automatically calculates the shortest route to
         *  the setpoint.
         * @param continuous Set to true turns on continuous, false turns off continuous
         */
        public void setContinuous(boolean continuous) {
            m_continuous = continuous;
        }

        /**
         *  Set the PID controller to consider the input to be continuous,
         *  Rather then using the max and min in as constraints, it considers them to
         *  be the same point and automatically calculates the shortest route to
         *  the setpoint.
         */
        public void setContinuous() {
            this.setContinuous(true);
        }

        /**
         * Sets the maximum and minimum values expected from the input.
         *
         * @param minimumInput the minimum value expected from the input, always positive
         * @param maximumInput the maximum value expected from the output, always positive
         */
        public void setInputRange(double minimumInput, double maximumInput)
        {
            m_minimumInput = Math.abs(minimumInput);
            m_maximumInput = Math.abs(maximumInput);
            setSetpoint(m_setpoint);
        }

        /**
         * Sets the minimum and maximum values to write.
         *
         * @param minimumOutput the minimum value to write to the output, always positive
         * @param maximumOutput the maximum value to write to the output, always positive
         */
        public void setOutputRange(double minimumOutput, double maximumOutput)
        {
            m_minimumOutput = Math.abs(minimumOutput);
            m_maximumOutput = Math.abs(maximumOutput);
        }

        /**
         * Set the setpoint for the PIDController
         * @param setpoint the desired setpoint
         */
        public void setSetpoint(double setpoint)
        {
            int     sign = 1;

            if (m_maximumInput > m_minimumInput)
            {
                if (setpoint < 0) sign = -1;

                if (Math.abs(setpoint) > m_maximumInput)
                    m_setpoint = m_maximumInput * sign;
                else if (Math.abs(setpoint) < m_minimumInput)
                    m_setpoint = m_minimumInput * sign;
                else
                    m_setpoint = setpoint;
            }
            else
                m_setpoint = setpoint;
        }

        /**
         * Returns the current setpoint of the PIDController
         * @return the current setpoint
         */
        public double getSetpoint() {
            return m_setpoint;
        }

        /**
         * Retruns the current difference of the input from the setpoint
         * @return the current error
         */
        public synchronized double getError() {
            return m_error;
        }

        /**
         * Set the percentage error which is considered tolerable for use with
         * OnTarget. (Input of 15.0 = 15 percent)
         * @param percent error which is tolerable
         */
        public void setTolerance(double percent) {
            m_tolerance = percent;
        }

        /**
         * Return true if the error is within the percentage of the total input range,
         * determined by setTolerance. This assumes that the maximum and minimum input
         * were set using setInputRange.
         * @return true if the error is less than the tolerance
         */
        public boolean onTarget()
        {
            return (Math.abs(m_error) < Math.abs(m_tolerance / 100 * (m_maximumInput - m_minimumInput)));
        }

        /**
         * Begin running the PIDController
         */
        public void enable() {
            m_enabled = true;
        }

        /**
         * Stop running the PIDController.
         */
        public void disable() {
            m_enabled = false;
        }

        /**
         * Reset the previous error,, the integral term, and disable the controller.
         */
        public void reset()
        {
            disable();
            m_prevError = 0;
            m_totalError = 0;
            m_result = 0;
        }

        /**
         * Set the input value to be used by the next call to performPID().
         * @param input Input value to the PID calculation.
         */
        public void setInput(double input)
        {
            int     sign = 1;

            if (m_maximumInput > m_minimumInput)
            {
                if (input < 0) sign = -1;

                if (Math.abs(input) > m_maximumInput)
                    m_input = m_maximumInput * sign;
                else if (Math.abs(input) < m_minimumInput)
                    m_input = m_minimumInput * sign;
                else
                    m_input = input;
            }
            else
                m_input = input;
        }
        
        
    }
}
