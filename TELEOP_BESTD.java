package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.io.Serializable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@TeleOp (name = "TELEOP_BESTD (Blocks to Java)")
public class TELEOP_BESTD extends LinearOpMode {
  double fixedtheta=0;//find this
  double rotater=0;
  double motifs=0;
  double deltaX=0;
  double startx=0;
  double runtime=0;
  double starty=0;
  double deltaY=0;
  boolean USE_WEBCAM;
  AprilTagProcessor myAprilTagProcessor;
  Position cameraPosition;
  YawPitchRollAngles cameraOrientation;
  VisionPortal myVisionPortal;
  double rangeB=0; 
  public double offsetX=9.129397076417323;
  public double offsetY=5.1496063;//fine tune this
  double pointAx=-16.5354;//tune this in inches times -1
  double pointBx=0.0001;
  double pointCx=0;
  double pointAy=0.0001;
  double pointBy=16.5354;// robot height of camera in inches
  double pointCy=38.759843+offsetY;

  double currentpositionY=0;
  double currentpositionX=0; 
  double gravity=386.08858267717;
  boolean thisExpUp;
  boolean thisExpDn;
  boolean thisGainUp;
  boolean thisGainDn;
  boolean lastExpUp;
  boolean lastExpDn;
  boolean lastGainUp;
  boolean lastGainDn;
  double onerev=383.6;
  double desiredspeed=0;
  double intialspeed=0; //intial speed before change measured in ticks adjust value
  private DcMotor backleft;
  private DcMotor backright;
  private DcMotor frontright;
  private DcMotor frontleft;
  private Servo pivotintake;
  private Servo pivotintakeA;
  private Servo shooterholder;
  private Servo artifactholder;
  private CRServo belt;
  private DcMotor intake;
  private DcMotor shooterwheelA;
  private DcMotor shooterwheelB;
  private CRServo holder;
  private DcMotor X;
  
  public double change=10; //adjust value
  public double degree1=0.5;
  public double degree2=0;
  public double latchopen=0.5; 
  public double latchclose=0;
  public double artifactholderopen=0.5; // adjust value in the future
  public double artifactholderclose=0; // adjust value in the future
  public double shooterholderopen=0.5; // adjust value in the future
  public double shooterholderclose=0; // adjust value in the future 
  public double beltspeed1=-1; // adjust value in the future
  public double beltspeed2=1; // adjust value in the future
  public double Circumference=76.8*Math.PI; //in mm
  
  double[] motif={0,0,0,0};
  double[] correctmotif={0,0,0,0};
  double red=0;
  double green=0;
  double blue=0;
  double test=0;
  double sense=0;
  double Xa=0;
  double Ya=0;
  double y=0;
  double x=0;
  double Za=0;
  double Pitch=0;
  double Yaw=0;
  double Roll=0;
  double Ycenter=0;
  double Xcenter=0;
  int i=0;
  double A=0;
  double B=0;
  double C=0;
  public class FeedforwardB{
      public double kSB=0;//find this value
      public double kAB=0;//find this value
      public double kVB=0;//find this value
      double DesiredVB=0;//find this value
      double DesiredAB=0;//find this value
      
      public double feedforwardtermB(double DesiredVB,double DesiredAB,double kSB,double kVB,double kAB){
           this.kSB=kSB;
           this.kVB=kVB;
           this.kAB=kAB;
           double outputffB=kSB*sign(DesiredVB)+kVB*DesiredVB+kAB*DesiredAB;
           return outputffB;
      }
  }
  public double sign(double v){
         if(v>0){
             return 1; 
         }
         if(v==0){
             return 0;
         }
         if(v<0){
            return -1;
         }
  } 
  public class PIDCONTOLLERshooterB{
      public double kpshooterB; //find this value
      public double kishooterB; //find this value
      public double kdshooterB;//find this value
      double previousErrorshooterB=0;
      double intergralshooterB=0; //assign a value in the future to intergral
      double minOutputshooterB=0; //assign a value in the future to minoutput
      double maxOutputshooterB=0; //assign a value in the future to maxoutput

      public double PIDshooterB(double kpshooterB, double kishooterB, double kdshooterB, double shooterB1, double shooterB2, double shooterB3){
    
         this.kpshooterB=kpshooterB;
         this.kishooterB=kishooterB;
         this.kdshooterB=kdshooterB;
         double outputshooterBa = kpshooterB * shooterB1 + kishooterB * shooterB3 + kdshooterB * shooterB2;
         return outputshooterBa;
       }
       public double calcshooterB(double targetshooterB,double currentshooterB){
          double errorshooterB = targetshooterB - currentshooterB;
          double integralmaxB=1000;//update this value if needed
          double timeBs=0.2; //update if needed
          double integralshooterB =+ errorshooterB*timeBs;
          if(integralshooterB>integralmaxB){
              integralshooterB=integralmaxB;
          }
          if(integralshooterB<(integralmaxB*-1)){
              integralshooterB=integralmaxB*-1;
          }
          double derivativeshooterB = errorshooterB - previousErrorshooterB/timeBs;
          double outputshooterBa = PIDshooterB(kpshooterB, kishooterB, kdshooterB, errorshooterB, derivativeshooterB, integralshooterB);
          double outputshooterB = Math.max(minOutputshooterB, Math.min(maxOutputshooterB, outputshooterBa));
  
          double previousErrorshooterB = errorshooterB;
          return outputshooterB;
      }
      public void resetshooterB(){
         double previousErrorshooterB=0;
         double intergralshooterB=0;
      }

  }
  public class FeedforwardA{
      //do not tune
      public double kSA=0;
      public double kAA=0;
      public double kVA=0;
      double DesiredVA=0;
      double DesiredAA=0;

      public double feedforwardtermA(double DesiredVA,double DesiredAA,double kSA,double kVA,double kAA){
           this.kSA=kSA;
           this.kVA=kVA;
           this.kAA=kAA;
           double outputffA=kSA*sign(DesiredVA)+kVA*DesiredVA+kAA*DesiredAA;
           return outputffA;
          }
      }
  public class PIDCONTOLLERshooterA{
      //do not tune

      public double kpshooterA; 
      public double kishooterA; 
      public double kdshooterA;
      
      double previousErrorshooterA=0;
      double intergralshooterA=0; //assign a value in the future to intergral
      double minOutputshooterA=0; //assign a value in the future to minoutput
      double maxOutputshooterA=0; //assign a value in the future to maxoutput
      
      public double PIDshooterA(double kpshooterA, double kishooterA, double kdshooterA, double shooterA1, double shooterA2, double shooterA3){
         
         this.kpshooterA=kpshooterA;
         this.kishooterA=kishooterA;
         this.kdshooterA=kdshooterA;
         double outputshooterAa = kpshooterA * shooterA1 + kishooterA * shooterA3 + kdshooterA * shooterA2;
         return outputshooterAa;
       }
       public double calcshooterA(double targetshooterA,double currentshooterA){
          double errorshooterA = targetshooterA - currentshooterA;
          
          double integralmaxA=1000;//update this value if needed
          double timeAs=0.2; //update if needed
          double integralshooterA =+ errorshooterA*timeAs;
          if(integralshooterA>integralmaxA){
              integralshooterA=integralmaxA;
          }  
          if(integralshooterA<(integralmaxA*-1)){
              integralshooterA=integralmaxA*-1;
          }
          double derivativeshooterA = errorshooterA - previousErrorshooterA/timeAs;
          double outputshooterAa = PIDshooterA(kpshooterA, kishooterA, kdshooterA, errorshooterA, derivativeshooterA, integralshooterA);
          double outputshooterA = Math.max(minOutputshooterA, Math.min(maxOutputshooterA, outputshooterAa));
  
          double previousErrorshooterA = errorshooterA;
          return outputshooterA;
      }    
      public void resetshooterA(){
         double previousErrorshooterA=0;
         double intergralshooterA=0;
      }
  }

  public double answervelocity(double heightofgoal, double g, double theta){
       return (Math.sqrt(2*g*heightofgoal)/Math.sin(theta));
  }
  @Override
  public void runOpMode() {
    //private ElapsedTime runtime = new ElapsedTime();
    pivotintake = hardwareMap.get(Servo.class, "pivotintake");
    pivotintakeA = hardwareMap.get(Servo.class, "pivotintakeA");
    belt = hardwareMap.get(CRServo.class, "belt");
    USE_WEBCAM = true;
  
    double speedOfintakeOff=0;
    double speedOfintakeOn=0.8;
    double yAxis= -gamepad2.left_stick_y;
    double xAxis= gamepad2.left_stick_x;
    double zAxis= gamepad1.right_stick_y;
    
    double x=0;
    double y=0;
    double turn=0;
    double backleft_A;
    double swerve_A;
    double swerve_B;
    double backright_A;
    double frontleft_A;
    double frontright_A;
    shooterholder= hardwareMap.get(Servo.class, "shooterholder");
    artifactholder= hardwareMap.get(Servo.class, "artifactholder");
    shooterwheelA = hardwareMap.get(DcMotor.class, "shooterwheelA");
    shooterwheelB = hardwareMap.get(DcMotor.class, "shooterwheelB");
    holder = hardwareMap.get(CRServo.class, "holder");
    X=hardwareMap.get(DcMotor.class, "X");
    intake = hardwareMap.get(DcMotor.class, "intake");
    backleft = hardwareMap.get(DcMotor.class, "backleft");
    backright = hardwareMap.get(DcMotor.class, "backright");
    frontright = hardwareMap.get(DcMotor.class, "frontright");
    frontleft = hardwareMap.get(DcMotor.class, "frontleft");  
    intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    X.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    shooterwheelA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    shooterwheelB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    shooterwheelB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    shooterwheelA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    X.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    initAprilTag();
    getCameraSetting();
    myExposure = 10;
    myGain = 50;
    setManualExposure();
    telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
    telemetry.addData(">", "Touch START to start OpMode");
    telemetry.update();
    waitForStart();
    runtime.reset();
    while (opModeIsActive()) {
        
      y = gamepad2.left_stick_y;
      x = gamepad2.left_stick_x;
      turn = gamepad2.right_stick_x;
      rotater= gamepad1.left_stick_y;
      swerve_A=-turn;
      swerve_B=turn;
      frontleft_A = y - x ;
      frontright_A = (y + x) ;
      backleft_A = (y + x) ;
      backright_A = (y - x) ;
      if(gamepad1.y){
         artifactholder.setPosition(artifactholderopen);
      }
      if(gamepad1.x){
         artifactholder.setPosition(artifactholderclose);
      }
      if(gamepad1.dpad_up){
         shooterholder.setPosition(shooterholderclose);
      }
      if(gamepad1.dpad_down){
         shooterholder.setPosition(shooterholderopen);
      }     
      if(gamepad1.dpad_left){
        pivotintakeA.setPosition(latchopen);
      }
      if(gamepad1.dpad_right){
        pivotintakeA.setPosition(latchclose);
      }
      if(gamepad1.a){
        pivotintake.setPosition(degree1);
      }
      if(gamepad1.b){
        pivotintake.setPosition(degree2);
      }
      if(gamepad2.y){
         holder.setPower(speedOfintakeOn);
      }
      if(gamepad2.x){
         holder.setPower(speedOfintakeOff);
      }
      if(gamepad1.left_bumper){
           intake.setPower(speedOfintakeOn); 
           
      }
      if(gamepad1.right_bumper){
        
          intake.setPower(speedOfintakeOff); 
          

      }

      

      if(rotater<0){
        belt.setPower(beltspeed1);
      }
      if(rotater>0){
        belt.setPower(beltspeed2);
      }
      if(rotater==0){
        belt.setPower(0);
      }

      if (gamepad2.right_bumper) {
          double precision=1-gamepad2.left_trigger;    
          swerve_A=precision*(swerve_A);
          swerve_B=precision*(swerve_B);
          frontleft_A=precision*(frontleft_A);
          frontright_A=precision*(frontright_A);
          backleft_A=precision*(backleft_A);
          backright_A=precision*(backright_A);
      }

      if (gamepad2.left_bumper) {
          double precision=0.3-(gamepad2.left_trigger*0.3); 
          swerve_A=precision*(swerve_A);
          swerve_B=precision*(swerve_B);
          frontleft_A=precision*(frontleft_A);
          frontright_A=precision*(frontright_A);
          backleft_A=precision*(backleft_A);
          backright_A=precision*(backright_A);
      }
      if(!gamepad2.left_bumper && !gamepad2.right_bumper){
          double precision=0.6-(gamepad2.left_trigger*0.6);                  
          swerve_A=precision*(swerve_A);
          swerve_B=precision*(swerve_B);                                           
          frontleft_A=precision*(frontleft_A);
          frontright_A=precision*(frontright_A);
          backleft_A=precision*(backleft_A);
          backright_A=precision*(backright_A);
      }
                 

      if(gamepad2.a){
              frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
              frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
              backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
              backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); 
      }
      else if(gamepad2.b){
              frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);    
              frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); 
              backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);     
              backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      }
      else{
              frontright.setPower(-frontright_A);        
              frontleft.setPower(-frontleft_A);          
              backleft.setPower(backleft_A);            
              backright.setPower(backright_A);
      }
      
      List<AprilTagDetection> myAprilTagDetections;
      
      AprilTagDetection myAprilTagDetection;
    
      // Get a list of AprilTag detections.
      myAprilTagDetections = myAprilTagProcessor.getDetections();
      telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
  
      for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
        myAprilTagDetection = myAprilTagDetection_item;
      // Display info about the detection.
        telemetry.addLine("");
        
        if (myAprilTagDetection.metadata != null) {
          telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
         // Only use tags that don't have Obelisk in them since Obelisk tags don't have valid location data
          test=myAprilTagDetection.id;
          motifs=test;
          
          if (!contains(myAprilTagDetection.metadata.name, "Obelisk")) {
            currentpositionY=intake.getCurrentPosition();
            currentpositionX=X.getCurrentPosition(); 
            Ya=Math.round(myAprilTagDetection.robotPose.getPosition().y*10);
            Xa=Math.round(myAprilTagDetection.robotPose.getPosition().x*10);
            Za=Math.round(myAprilTagDetection.robotPose.getPosition().z*10);
            Pitch=Math.round(myAprilTagDetection.robotPose.getOrientation().getPitch()*10);
            Roll=Math.round(myAprilTagDetection.robotPose.getOrientation().getRoll()*10);
            Yaw=Math.round(myAprilTagDetection.robotPose.getOrientation().getYaw()*10);
            rangeB=Math.sqrt(Math.pow(((startx - currentpositionX) * Math.PI * 1.25984) / 2000, 2) + Math.pow(((starty - currentpositionY) * Math.PI * 1.25984) / 2000, 2));
            pointCx=rangeB+offsetX;
            
            //telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().z, 6, 1) + "  (inch)");
            telemetry.addLine("XYZ " + Xa/10 + " " + Ya/10 + " " + Za/10 + "  (inch)");
            telemetry.addLine("PRY " +  Pitch/10 + " " + Roll/10 + " " + Yaw/10 + " \u03B8 (deg)");
            
          }
        } else {
           telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
           telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
         }

      }
      telemetry.addLine("");
      telemetry.addLine("key:");
      telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
      telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
      if(motifs!=20 && motifs!=24){
       if(motifs==23){
         correctmotif[0]=2;
         correctmotif[1]=2;
         correctmotif[2]=1;
       }
       if(motifs==21){
        correctmotif[0]=1;
        correctmotif[1]=2;
        correctmotif[2]=2;
       }
       if(motifs==22){
        correctmotif[0]=2;
        correctmotif[1]=1;
        correctmotif[2]=2;
       }
      

        

      }
      intialspeed=answervelocity(pointCy, gravity, fixedtheta);
      if(gamepad1.right_trigger>0){
            desiredspeed=intialspeed-(change*gamepad1.right_trigger);
      }
      if(gamepad1.left_trigger>0){
            desiredspeed=intialspeed+(change*gamepad1.left_trigger);
      }
      if(gamepad1.left_trigger>0 && gamepad1.right_trigger>0){
            desiredspeed=intialspeed;
      }
      if(gamepad1.right_stick_button){
          feedforwardtermB termB=feedforwardtermB();
          PIDCONTOLLERshooterB ShooterB=PIDCONTOLLERshooterB();
          double currentspeedB=((shooterwheelB.getCurrentPosition()/383.6)*Circumference*(96/32))/runtime.seconds();
          double VelocityB=0;//tune this
          double AccelerationB=0;//tune this
          double KSshooterB=0;//tune this
          double KVshooterB=0;//tune this
          double KAshooterB=0;//tune this
          double feedforwardB=termB.feedforwardtermB(VelocityB, AccelerationB, KSshooterB, KVshooterB, KAshooterB);
          
 
          double speedB=shooterB.calcshooterB(desiredspeed,currentspeedB)+feedforwardB; 
          shooterwheelB.setPower(speedB);
          shooterwheelA.setPower(-speedB);
      }
      telemetry.update();
       
      lastExpUp = thisExpUp;
      lastExpDn = thisExpDn;
      lastGainUp = thisGainUp;
      lastGainDn = thisGainDn;
      sleep(20);
        
       

        
      

      
      
    }
  }
  
  /**
   * Initialize AprilTag Detection.
   */
  private void initAprilTag() {
    AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    VisionPortal.Builder myVisionPortalBuilder;

    // First, create an AprilTagProcessor.Builder.
    myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    myAprilTagProcessorBuilder.setCameraPose(cameraPosition, cameraOrientation);
    // Create an AprilTagProcessor by calling build.
    myAprilTagProcessor = myAprilTagProcessorBuilder.build();

    myVisionPortalBuilder = new VisionPortal.Builder();
    if (USE_WEBCAM) {
      // Use a webcam.
      myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
    } else {
      myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
    }
    // Add myAprilTagProcessor to the VisionPortal.Builder.
    myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
    // Create a VisionPortal by calling build.
    myVisionPortal = myVisionPortalBuilder.build();
  }




  private boolean contains(String stringToSearch, String containText) {
    if (stringToSearch.indexOf(containText) + 1 == 0) {
      return false;
    }
    return true;
  }
  private void getCameraSetting() {

    waitForCamera();
    // Get camera control values unless we are stopping.
    if (!isStopRequested()) {
      myExposureControl = myVisionPortal.getCameraControl(ExposureControl.class);
      minExposure = 10;
      maxExposure = 30;
      myGainControl = myVisionPortal.getCameraControl(GainControl.class);
      minGain = 0;
      maxGain = 100;
    }
  }


  private void waitForCamera() {
    if (!myVisionPortal.getCameraState().equals(VisionPortal.CameraState.STREAMING)) {
      telemetry.addData("Camera", "Waiting");
      telemetry.update();
      while (!isStopRequested() && !myVisionPortal.getCameraState().equals(VisionPortal.CameraState.STREAMING)) {
        sleep(20);
      }
      telemetry.addData("Camera", "Ready");
      telemetry.update();
    }
  }
}


  
