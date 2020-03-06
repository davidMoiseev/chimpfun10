package frc.robot;

public class Calibrations
{
    public final static boolean isCompBot=true;

    public class auton{
        public static final boolean trench = false;
    }

    public class CAN_ID {
        public final static int pigeon = 30;
        public final static int driveLeft1 = 4;
        public final static int driveLeft2 = 3;
        public final static int driveRight1 = 1;
        public final static int driveRight2 = 2;
        public final static int shooter1 = 5;
        public final static int shooter2 = 6;
        public final static int indexer = 12;
        public final static int conveyor = 13;
        public final static int carousel = 11;
        public final static int intakeMotor1 = 9;
        public final static int intakeMotor2 = 10;
        public final static int intakeLifter = 14;
        public final static int pcm = 0;
        public final static int CANifier = 20;
        public final static int leftClimber = 7;
        public final static int rightClimber = 8;

     }
    public class shooter_PID{
        public final static double kP = 1.1e-3; 
        public final static double kI = 3e-6;
        public final static double kD = 2e-7; 
        public final static double kIz = 200; 
        public final static double kFF = 0.000137862*1.6; 
        public final static double kMaxOutput = 1; 
        public final static double kMinOutput = -0.15;
    }
    public class intake_PID{
        public final static double kP = 1e-3; 
        public final static double kI = 3e-6;
        public final static double kD = 2e-7; 
        public final static double kIz = 225; 
        public final static double kFF = 0.000137862*1.08; 
        public final static double kMaxOutput = 1; 
        public final static double kMinOutput = -0.5;
        public final static int packagePosition = 500;
        public final static int groundPosition = 0;
        public final static int limelitePosition = 250;
        public final static int middlePosition = 350;
    }
    public class ballSuperviserVals{
        public final static int shooterCurrentLimit = 40;
        public final static int intakeArmCurrentLimit = 30;
        public final static double intakeStandardPower = 0.75;
        public final static double indexerPower = 1;
        public final static double ledCycleTime = 15;
        public final static double ledDutyCycle = 0.5;
    }
    public class Vision {
        public static final double kP = 0.027;
        public static final double kI = 0.06;
        public static final double kD = 0.001;
        public static final double kPDistance = 0.05;
        public static final double kIDistance = 0;
        public static final double kDistance = 0;
        public static final double kLimelightDistanceFromFront = 0; // In recording distance minus distance by how far back limelight is from front of robot
        public static final double deadband = .04; 
        public static final double kHeight = (2.7114 - .17); // Height of Target - Height of Camera
        public static final double kMountedAngle = 41.1;
    }
    public class climberCals{
        public static final int rachetID = 4;
        public static final double kP = 0.05;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final int kIZ = 0;
        public static final int acc = 1600/2;
        public static final int vel = 1600/2;
        public static final double ticksPerInch = 118;
        public static final double upperLimit = 70; //Upper Hight Limit    
        public static final double maxDelta = 12; //Max Target Change
        
    }

    public class ArmPositions{
        public static final double packagedAngle = 90;  
        public static final double trenchShotAngle = 63;
        public static final double autoShotAngle = 62;
        public static final double wallShotAngle = 10;
        public static final double groundPickupAngle = 0.1;
        public static final double wofAngle = 59.45; //7015 tiks
        public static final double maxTicks = 7450; //7700 is max
    }

    public class DRIVE_CONSTANTS {  //6.5 gear ratio
        public static final double ticksPerMeter = 38825; //38449.2;
        public static final double trackWidth = 0.5925;   //meters
		public static final double YawCorrection_kP = 0.025;
    }
            //wheel diameter 0.0127
    public class AUTO_CONTROLLERS {
        public static final double velocityPIDkp = 0.0; //0.000183 ; 
        public static final double velocityPIDki = 0.0;
        public static final double velocityPIDkd = 0.0;
        public static final double velocityPIDkf = 0.0603;
        public static final double ramseteB = 3;
        public static final double ramseteTheta = 0.7;
        public static final double ffkS = 0.276; //0.577;
        public static final double ffkV = 0.276; //2.36;        //2.36 * 10 *1023 * 0.0635 *2 *pi /6.5 / 12 /4096
        public static final double ffkA = 0.0391; //0.196;
		public static final double voltagekP = 3.72; //3.72;
		public static final double voltagekI = 0;
		public static final double voltagekD = 0;
	
}
    public class ARM{
        public static final double kP = 1.5;
        public static final double kI = 0.01  * 2 /3;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final int kIZ = 480;
        public static final int acc = 1600;
        public static final int vel = 1600;
        public static final double ticksPerDegree = 118;
        public static final double maxGravityFF = 0.139;
        public static final double measuredTicksHorizontal = 82;
        public static final double resetCurrentDraw = 20;  //amps, log motion magic to get value      
        public static final double ticksAt90 = 300; //configure  by measuring ticks when arm is level using level
        public static final double limelightHeightAtArm90 = 0.426; //configure, from ground  //difference on comp bot becausw of hard stop is 1.51 degrees
        public static final double lengthOfArmToLimelight = 0.62;
    }

    public class hardware{
        public static final boolean longPistonExtend = isCompBot?false:true;
        public static final boolean shortPistonExtend = isCompBot;
        public static final boolean indexerInvert = isCompBot?false:true;
    }


}