package frc.robot;

public class Calibrations
{

    public class CAN_ID {
        public final static int pigeon = 20;
        public final static int driveLeft1 = 3;
        public final static int driveLeft2 = 4;
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
        public final static double indexerPower = 0.40;
    }
    public class Vision {
        public static final double kP = 0.027;
        public static final double kI = 0.03;
        public static final double kD = 0.001;
        public static final double kPDistance = 0.05;
        public static final double kIDistance = 0;
        public static final double kDistance = 0;
        public static final double kLimelightDistanceFromFront = 0; // In recording distance minus distance by how far back limelight is from front of robot
        public static final double deadband = .05;
        public static final double kHeight = (2.7114 - .17); // Height of Target - Height of Camera
        public static final double kMountedAngle = 41.1;
    }

}