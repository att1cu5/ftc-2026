package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous (name = "auto (Blocks to Java)")
public class auto extends LinearOpMode {
    IMU imu;
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
    double myYAW=0;
    double Roll=0;
    double Ycenter=0;
    double Xcenter=0;
    int i=0;
    double A=0;
    double B=0;
    double C=0;
    class PIDcontrollerCang{
        public double kpCang=0;
        public double kiCang=0;
        public double kdCang=0;
        public double setPointCang=0;
        double lasterrorCAng=0;
        double outputCAng=0;
        double IntergralSumCang=0;
        public PIDcontrollerCang(double kpCang,double kiCang,double kdCang,double setPointCang){
            this.kpCang=kpCang;
            this.kiCang=kiCang;
            this.kdCang=kdCang;
            this.setPointCang=setPointCang;
        }
        public double calcCang(double MesurementCang){
            double errorCang=setPointCang-MesurementCang;
            double kpCangTerm=kpCang*errorCang;
            IntergralSumCang+=errorCang;
            double kiCangTerm=kiCang*IntergralSumCang;
            double KdCangTerm=(errorCang-lasterrorCAng);
            double KdCangTermA=kdCang*KdCangTerm;
            double outputCAng=kiCangTerm+KdCangTermA+kpCangTerm;
            lasterrorCAng=errorCang;
            return outputCAng;
        }
    }


}
public class PIDcontrollerDang{
    public double kpDang=0;
    public double kiDang=0;
    public double kdDang=0;
    public double setPointDang=0;
    double lasterrorDAng=0;
    double outputDAng=0;
    double IntergralSumDang=0;
    public PIDcontrollerDang(double kpDang,double kiDang,double kdDang,double setPointDang){
        this.kpDang=kpDang;
        this.kiDang=kiDang;
        this.kdDang=kdDang;
        this.setPointDang=setPointDang;
    }
    public double calcDang(double MesurementDang){
        double errorDang=setPointDang-MesurementDang;
        double kpDangTerm=kpDang*errorDang;
        IntergralSumDang+=errorDang;
        double kiDangTerm=kiDang*IntergralSumDang;
        double KdDangTerm=(errorDang-lasterrorDAng);
        double KdDangTermA=kdDang*KdDangTerm;
        double outputDAng=kiDangTerm+KdDangTermA+kpDangTerm;
        lasterrorDAng=errorDang;
        return outputDAng;
    }
}


public class PIDcontrollerAang{
    public double kpAang=0;
    public double kiAang=0;
    public double kdAang=0;
    public double setPointAang=0;
    double lasterrorAAng=0;
    double outputAAng=0;
    double IntergralSumAang=0;
    public PIDcontrollerAang(double kpAang,double kiAang,double kdAang,double setPointAang){
        this.kpAang=kpAang;
        this.kiAang=kiAang;
        this.kdAang=kdAang;
        this.setPointAang=setPointAang;
    }
    public double calcAang(double MesurementAang){
        double errorAang=setPointAang-MesurementAang;
        double kpAangTerm=kpAang*errorAang;
        IntergralSumAang+=errorAang;
        double kiAangTerm=kiAang*IntergralSumAang;
        double KdAangTerm=(errorAang-lasterrorAAng);
        double KdAangTermA=kdAang*KdAangTerm;
        double outputAAng=kiAangTerm+KdAangTermA+kpAangTerm;
        lasterrorAAng=errorAang;
        return outputAAng;
    }
}


public class PIDcontrollerEang{
    public double kpEang=0;
    public double kiEang=0;
    public double kdEang=0;
    public double setPointEang=0;
    double lasterrorEAng=0;
    double outputEAng=0;
    double IntergralSumEang=0;
    public PIDcontrollerEang(double kpEang,double kiEang,double kdEang,double setPointEang){
        this.kpEang=kpEang;
        this.kiEang=kiEang;
        this.kdEang=kdEang;
        this.setPointEang=setPointEang;
    }
    public double calcEang(double MesurementEang){
        double errorEang=setPointEang-MesurementEang;
        double kpEangTerm=kpEang*errorEang;
        IntergralSumEang+=errorEang;
        double kiEangTerm=kiEang*IntergralSumEang;
        double KdEangTerm=(errorEang-lasterrorEAng);
        double KdEangTermA=kdEang*KdEangTerm;
        double outputEAng=kiEangTerm+KdEangTermA+kpEangTerm;
        lasterrorEAng=errorEang;
        return outputEAng;
    }
}

public class FeedforwardEang{
    public double kSEang=0;//find this value
    public double kAEang=0;//find this value
    public double kVEang=0;//find this value
    double DesiredVEang=0;//find this value
    double DesiredAEang=0;//find this value

    public double feedforwardtermEang(double DesiredVEang,double DesiredAEang,double kSEang,double kVEang,double kAEang){
        this.kSEang=kSEang;
        this.kVEang=kVEang;
        this.kAEang=kAEang;
        double outputffEang=kSEang*Math.abs(DesiredVEang)+kVEang*DesiredVEang+kAEang*DesiredAEang;
        return outputffEang;
    }
}
public class FeedforwardDang{
    public double kSDang=0;//find this value
    public double kADang=0;//find this value
    public double kVDang=0;//find this value
    double DesiredVDang=0;//find this value
    double DesiredADang=0;//find this value

    public double feedforwardtermDang(double DesiredVDang,double DesiredADang,double kSDang,double kVDang,double kADang){
        this.kSDang=kSDang;
        this.kVDang=kVDang;
        this.kADang=kADang;
        double outputffDang=kSDang*Math.abs(DesiredVDang)+kVDang*DesiredVDang+kADang*DesiredADang;
        return outputffDang;
    }
}
public class FeedforwardAang{
    public double kSAang=0;//find this value
    public double kAAang=0;//find this value
    public double kVAang=0;//find this value
    double DesiredVAang=0;//find this value
    double DesiredAAang=0;//find this value

    public double feedforwardtermAang(double DesiredVAang,double DesiredAAang,double kSAang,double kVAang,double kAAang){
        this.kSAang=kSAang;
        this.kVAang=kVAang;
        this.kAAang=kAAang;
        double outputffAang=kSAang*Math.abs(DesiredVAang)+kVAang*DesiredVAang+kAAang*DesiredAAang;
        return outputffAang;
    }
}
public class FeedforwardCang{
    public double kSCang=0;//find this value
    public double kACang=0;//find this value
    public double kVCang=0;//find this value
    double DesiredVCang=0;//find this value
    double DesiredACang=0;//find this value

    public double feedforwardtermCang(double DesiredVCang,double DesiredACang,double kSCang,double kVCang,double kACang){
        this.kSCang=kSCang;
        this.kVCang=kVCang;
        this.kACang=kACang;
        double outputffCang=kSCang*Math.abs(DesiredVCang)+kVCang*DesiredVCang+kACang*DesiredACang;
        return outputffCang;
    }
}
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
        double outputffB=kSB*Math.abs(DesiredVB)+kVB*DesiredVB+kAB*DesiredAB;
        return outputffB;
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

//frontrightwheel
public class PIDCONTOLLERshooterA{


    public double kpshooterA; //find this value
    public double kishooterA; //find this value
    public double kdshooterA;//find this value

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
public class FeedforwardA{
    double kSA=0;//find this value
    double kAA=0;//find this value
    double kVA=0;//find this value
    double DesiredVA=0;//find this value
    double DesiredAA=0;//find this value

    public double feedforwardtermA(double DesiredVA,double DesiredAA,double kSA,double kVA,double kAA){
        this.kSA=kSA;
        this.kVA=kVA;
        this.kAA=kAA;
        double outputffA=kSA*Math.abs(DesiredVA)+kVA*DesiredVA+kAA*DesiredAA;
        return outputffA;
    }
}
//frontleftwheel
public class PIDCONTOLLERshooterC{


    public double kpshooterC; //find this value
    public double kishooterC; //find this value
    public double kdshooterC;//find this value

    double previousErrorshooterC=0;
    double intergralshooterC=0; //assign a value in the future to intergral
    double minOutputshooterC=0; //assign a value in the future to minoutput
    double maxOutputshooterC=0; //assign a value in the future to maxoutput

    public double PIDshooterC(double kpshooterC, double kishooterC, double kdshooterC, double shooterC1, double shooterC2, double shooterC3){

        this.kpshooterC=kpshooterC;
        this.kishooterC=kishooterC;
        this.kdshooterC=kdshooterC;
        double outputshooterCa = kpshooterC * shooterC1 + kishooterC * shooterC3 + kdshooterC * shooterC2;
        return outputshooterCa;
    }
    public double calcshooterC(double targetshooterC,double currentshooterC){
        double errorshooterC = targetshooterC - currentshooterC;

        double integralmaxC=1000;//update this value if needed
        double timeCs=0.2; //update if needed
        double integralshooterC =+ errorshooterC*timeCs;
        if(integralshooterC>integralmaxC){
            integralshooterC=integralmaxC;
        }
        if(integralshooterC<(integralmaxC*-1)){
            integralshooterC=integralmaxC*-1;
        }
        double derivativeshooterC = errorshooterC - previousErrorshooterC/timeCs;
        double outputshooterAC = PIDshooterC(kpshooterC, kishooterC, kdshooterC, errorshooterC, derivativeshooterC, integralshooterC);
        double outputshooterC = Math.max(minOutputshooterC, Math.min(maxOutputshooterC, outputshooterAC));

        double previousErrorshooterC = errorshooterC;
        return outputshooterC;
    }
    public void resetshooterC(){
        double previousErrorshooterC=0;
        double intergralshooterC=0;
    }
}
public class FeedforwardC{
    public double kSC=0;//find this value
    public double kAC=0;//find this value
    public double kVC=0;//find this value
    double DesiredVC=0;//find this value
    double DesiredAC=0;//find this value

    public double feedforwardtermC(double DesiredVC,double DesiredAC,double kSC,double kVC,double kAC){
        this.kSC=kSC;
        this.kVC=kVC;
        this.kAC=kAC;
        double outputffC=kSC*Math.abs(DesiredVC)+kVC*DesiredVC+kAC*DesiredAC;
        return outputffC;
    }
}
//backrightwheel
public class PIDCONTOLLERshooterD{


    public double kpshooterD; //find this value
    public double kishooterD; //find this value
    public double kdshooterD;//find this value

    double previousErrorshooterD=0;
    double intergralshooterD=0; //assign a value in the future to intergral
    double minOutputshooterD=0; //assign a value in the future to minoutput
    double maxOutputshooterD=0; //assign a value in the future to maxoutput

    public double PIDshooterD(double kpshooterD, double kishooterD, double kdshooterD, double shooterD1, double shooterD2, double shooterD3){

        this.kpshooterD=kpshooterD;
        this.kishooterD=kishooterD;
        this.kdshooterD=kdshooterD;
        double outputshooterDa = kpshooterD * shooterD1 + kishooterD * shooterD3 + kdshooterD * shooterD2;
        return outputshooterDa;
    }
    public double calcshooterD(double targetshooterD,double currentshooterD){
        double errorshooterD = targetshooterD - currentshooterD;

        double integralmaxD=1000;//update this value if needed
        double timeDs=0.2; //update if needed
        double integralshooterD =+ errorshooterD*timeDs;
        if(integralshooterD>integralmaxD){
            integralshooterD=integralmaxD;
        }
        if(integralshooterD<(integralmaxD*-1)){
            integralshooterD=integralmaxD*-1;
        }
        double derivativeshooterD = errorshooterD - previousErrorshooterD/timeDs;
        double outputshooterAD = PIDshooterD(kpshooterD, kishooterD, kdshooterD, errorshooterD, derivativeshooterD, integralshooterD);
        double outputshooterD = Math.max(minOutputshooterD, Math.min(maxOutputshooterD, outputshooterAD));

        double previousErrorshooterD = errorshooterD;
        return outputshooterD;
    }
    public void resetshooterD(){
        double previousErrorshooterD=0;
        double intergralshooterD=0;
    }
}
public class FeedforwardD{
    public double kSD=0;//find this value
    public double kAD=0;//find this value
    public double kVD=0;//find this value
    double DesiredVD=0;//find this value
    double DesiredAD=0;//find this value

    public double feedforwardtermD(double DesiredVD,double DesiredAD,double kSD,double kVD,double kAD){
        this.kSD=kSD;
        this.kVD=kVD;
        this.kAD=kAD;
        double outputffD=kSD*Math.abs(DesiredVD)+kVD*DesiredVD+kAD*DesiredAD;
        return outputffD;
    }
}
//backleftwheel
public class PIDCONTOLLERshooterE{


    public double kpshooterE; //find this value
    public double kishooterE; //find this value
    public double kdshooterE;//find this value

    double previousErrorshooterE=0;
    double intergralshooterE=0; //assign a value in the future to intergral
    double minOutputshooterE=0; //assign a value in the future to minoutput
    double maxOutputshooterE=0; //assign a value in the future to maxoutput

    public double PIDshooterE(double kpshooterE, double kishooterE, double kdshooterE, double shooterE1, double shooterE2, double shooterE3){

        this.kpshooterE=kpshooterE;
        this.kishooterE=kishooterE;
        this.kdshooterE=kdshooterE;
        double outputshooterEa = kpshooterE * shooterE1 + kishooterE * shooterE3 + kdshooterE * shooterE2;
        return outputshooterEa;
    }
    public double calcshooterE(double targetshooterE,double currentshooterE){
        double errorshooterE = targetshooterE - currentshooterE;

        double integralmaxE=1000;//update this value if needed
        double timeEs=0.2; //update if needed
        double integralshooterE =+ errorshooterE*timeEs;
        if(integralshooterE>integralmaxE){
            integralshooterE=integralmaxE;
        }
        if(integralshooterE<(integralmaxE*-1)){
            integralshooterE=integralmaxE*-1;
        }
        double derivativeshooterE = errorshooterE - previousErrorshooterE/timeEs;
        double outputshooterAE = PIDshooterE(kpshooterE, kishooterE, kdshooterE, errorshooterE, derivativeshooterE, integralshooterE);
        double outputshooterE = Math.max(minOutputshooterE, Math.min(maxOutputshooterE, outputshooterAE));

        double previousErrorshooterE = errorshooterE;
        return outputshooterE;
    }
    public void resetshooterE(){
        double previousErrorshooterE=0;
        double intergralshooterE=0;
    }
}
public class FeedforwardE{
    public double kSE=0;//find this value
    public double kAE=0;//find this value
    public double kVE=0;//find this value
    double DesiredVE=0;//find this value
    double DesiredAE=0;//find this value

    public double feedforwardtermE(double DesiredVE,double DesiredAE,double kSE,double kVE,double kAE){
        this.kSE=kSE;
        this.kVE=kVE;
        this.kAE=kAE;
        double outputffE=kSE*Math.abs(DesiredVE)+kVE*DesiredVE+kAE*DesiredAE;
        return outputffE;
    }
}
public double answervelocity(double heightofgoal, double g, double theta){
    return (Math.sqrt(2*g*heightofgoal)/Math.sin(theta));
}
@Override
public void runOpMode() {
    //private ElapsedTime runtime = new ElapsedTime();


    USE_WEBCAM = true;

    double speedOfintakeOff=0;
    double speedOfintakeOn=0.8;


    double x=0;
    double y=0;
    double turn=0;
    double backleft_A;
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

    waitForStart();
    runtime.reset();
    while (opModeIsActive()) {







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

        double intialspeedA=0;//CHANGE LATER
        feedforwardtermA termA=feedforwardtermA();
        PIDCONTOLLERshooterA ShooterA=PIDCONTOLLERshooterA();
        double currentspeedA=((frontright.getCurrentPosition()/383.6)*104*2*Math.PI*(96/32))/runtime.seconds();
        double VelocityA=0;//tune this
        double AccelerationA=0;//tune this
        double KSshooterA=0;//tune this
        double KVshooterA=0;//tune this
        double KAshooterA=0;//tune this
        double feedforwardA=termA.feedforwardtermA(VelocityA, AccelerationA, KSshooterA, KVshooterA, KAshooterA);
        double desiredspeedA=0;//CHANGE LATER
        double speedA=shooterA.calcshooterA(desiredspeedA,currentspeedA)+feedforwardA;
        double intialspeedC=0;//CHANGE LATER
        feedforwardtermC termC=feedforwardtermC();
        PIDCONTOLLERshooterC ShooterC=PIDCONTOLLERshooterC();
        double currentspeedC=((frontleft.getCurrentPosition()/383.6)*104*2*Math.PI*(96/32))/runtime.seconds();
        double VelocityC=0;//tune this
        double AccelerationC=0;//tune this
        double KSshooterC=0;//tune this
        double KVshooterC=0;//tune this
        double KAshooterC=0;//tune this
        double feedforwardC=termC.feedforwardtermC(VelocityC, AccelerationC, KSshooterC, KVshooterC, KAshooterC);
        double desiredspeedC=0;//CHANGE LATER
        double speedC=shooterC.calcshooterC(desiredspeedC,currentspeedC)+feedforwardC;
        double intialspeedD=0;//CHANGE LATER
        feedforwardtermD termD=feedforwardtermD();
        PIDCONTOLLERshooterD ShooterD=PIDCONTOLLERshooterD();
        double currentspeedD=((backright.getCurrentPosition()/383.6)*104*2*Math.PI*(96/32))/runtime.seconds();
        double VelocityD=0;//tune this
        double AccelerationD=0;//tune this
        double KSshooterD=0;//tune this
        double KVshooterD=0;//tune this
        double KAshooterD=0;//tune this
        double feedforwardD=termD.feedforwardtermD(VelocityD, AccelerationD, KSshooterD, KVshooterD, KAshooterD);
        double desiredspeedD=0;//CHANGE LATER
        double speedD=shooterD.calcshooterD(desiredspeedD,currentspeedD)+feedforwardD;
        double intialspeedE=0;//CHANGE LATER
        feedforwardtermE termE=feedforwardtermE();
        PIDCONTOLLERshooterE ShooterE=PIDCONTOLLERshooterE();
        double currentspeedE=((backleft.getCurrentPosition()/383.6)*104*2*Math.PI*(96/32))/runtime.seconds();
        double VelocityE=0;//tune this
        double AccelerationE=0;//tune this
        double KSshooterE=0;//tune this
        double KVshooterE=0;//tune this
        double KAshooterE=0;//tune this
        double feedforwardE=termE.feedforwardtermE(VelocityE, AccelerationE, KSshooterE, KVshooterE, KAshooterE);
        double desiredspeedE=0;//CHANGE LATER
        double speedE=shooterE.calcshooterE(desiredspeedE,currentspeedE)+feedforwardE;
        double desX=0;//tune later
        double desY=0; //tune later
        while(X.getCurrentPosition()<desX){
            frontright.setPower(speedA);
            frontleft.setPower(speedC);
            backleft.setPower(speedE);
            backright.setPower(speedD);
            if(X.getCurrentPosition()==desX){
                frontright.setPower(0);
                frontleft.setPower(0);
                backleft.setPower(0);
                backright.setPower(0);
                break;
            }
        }
        while(shooterwheelA.getCurrentPosition()<desY){
            frontright.setPower(speedA);
            frontleft.setPower(speedC);
            backleft.setPower(speedE);
            backright.setPower(speedD);
            if(shooterwheelA.getCurrentPosition()==desY){
                frontright.setPower(0);
                frontleft.setPower(0);
                backleft.setPower(0);
                backright.setPower(0);
                break;
            }
        }
        intialspeed=answervelocity(pointCy, gravity, fixedtheta);
        feedforwardtermB termB=feedforwardtermB();
        PIDCONTOLLERshooterB ShooterB=PIDCONTOLLERshooterB();
        double currentspeedB=((shooterwheelB.getCurrentPosition()/383.6)*Circumference*(96/32))/runtime.seconds();
        double VelocityB=0;//tune this
        double AccelerationB=0;//tune this
        double KSshooterB=0;//tune this
        double KVshooterB=0;//tune this
        double KAshooterB=0;//tune this
        double feedforwardB=termB.feedforwardtermB(VelocityB, AccelerationB, KSshooterB, KVshooterB, KAshooterB);

        double desiredspeed=0;//CHANGE LATER
        double speedB=shooterB.calcshooterB(desiredspeed,currentspeedB)+feedforwardB;
        shooterwheelB.setPower(speedB);
        shooterwheelA.setPower(speedB);


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

    }
}


