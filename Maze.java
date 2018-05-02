 // Error = measured distanced - actuall distance
// proportional = error * kp

// Every time through loop add errorTotal += error 
// Intergral = errorTotal * ki

// Derivate = rate of change of error 
// Everytime through loop lastError = error (at botttom of loop)
 // Derivative = error - lastError * kd

//integralActive == set to value (Don't start accumalting error until above this threshold

//Cap of integral term from growing too large with a limit value -> (50/ki)

//Power sent to motor = proportional + Integral + Derivative

//Have a wait time at the end of the loop
import ShefRobot.*;
import java.util.concurrent.TimeUnit;


public class Maze {
	

	static double maxSpeed = 50;
	static double errorTotal;
	static double lastError = 0;
	static double proportional;
	static double integral;
	static double derivative;
	static double targetDistance = 5; // cm
    static int defaultSpeed = 100;
	
	//Constants
	static double kp = 7.3;
	static double ki = 0;
	static double kd = 0;
	

	
	public static void main(String[] args) {
		
		//Create a robot object to use and connect to it
        Robot myRobot = new Robot("dia-lego-a1");
       
        //The robot is made of components which are themselves objects.
        //Create references to them as useful shortcuts
        Motor leftMotor = myRobot.getLargeMotor(Motor.Port.B);
        Motor rightMotor = myRobot.getLargeMotor(Motor.Port.C);
        UltrasonicSensor sensorL = myRobot.getUltrasonicSensor(Sensor.Port.S3);
        UltrasonicSensor sensorR = myRobot.getUltrasonicSensor(Sensor.Port.S2);
        
        
            
        while(true) {
            double sensorTarget = (sensorL.getDistance() + sensorR.getDistance())/2;
            double leftDistance = sensorL.getDistance();
            double rightDistance = sensorR.getDistance();

            if (leftDistance == Double.POSITIVE_INFINITY ||leftDistance ==  Double.NEGATIVE_INFINITY){
                leftDistance = 0;
            }
            if (rightDistance == Double.POSITIVE_INFINITY ||rightDistance == Double.NEGATIVE_INFINITY){
                rightDistance = 0;
            }

            double errorL = (leftDistance * 100) - sensorTarget; 
            double errorR = (rightDistance * 100) - sensorTarget; 
            // double errorRL = errorL + errorR;
            

            if(errorRL != 0){
                errorTotal += errorRL;
            }
            else{
                errorTotal = 0;
            }


            if(errorRL == 0){

                derivative = 0;
            }

            proportional = errorRL * kp;
            integral = errorTotal * ki;
            derivative = (errorRL - lastError) * kd;
            lastError = errorRL;

            double speed = Math.abs((proportional + integral + derivative) * 10);
            double motorSpeed = speed / 1.1;

            System.out.println("Error: "+errorRL);
            System.out.println("DISTANCE LEFT: "+sensorL.getDistance());
            System.out.println("DISTANCE RIGHT: "+sensorR.getDistance());

            if(speed >= maxSpeed){
                speed = maxSpeed;
            }

            
            if(errorRL == 0){
                leftMotor.setSpeed(defaultSpeed);
                rightMotor.setSpeed(defaultSpeed);
                leftMotor.forward();
                rightMotor.forward();
                System.out.println("go");

            }
              
            //Turn Left
            if(errorRL > 0){
                leftMotor.setSpeed(defaultSpeed);
                rightMotor.setSpeed((int)motorSpeed);
                leftMotor.forward();
                rightMotor.forward();
                System.out.println("leftTurn");
            }
            //Turn Right
            else{
                rightMotor.setSpeed(defaultSpeed);
                leftMotor.setSpeed((int)motorSpeed);
                leftMotor.forward();
                rightMotor.forward();
                System.out.println("rightTurn");

            }
        
        
        }
    


		

		// //Disconnect from the Robot
  //       myRobot.close();
		

	}


}
