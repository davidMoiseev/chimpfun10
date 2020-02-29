package frc.robot.Autos;
import frc.robot.AutoRoutineRunner.BallModes;
import frc.robot.AutoRoutineRunner.autoModes;

public class SimpleRoutine extends AutoRoutineBase{

    public static int numberOfSteps = 2;
	
   
    public static class state1{
   
        public static autoModes driveMode = autoModes.autoAlign;
        public static BallModes ballMode = BallModes.primingAutoShot;
       
    }

    public static class state2{

        public static autoModes driveMode = autoModes.nothing;
        public static BallModes ballMode = BallModes.shooting;
    }



    

    

	

}
