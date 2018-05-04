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

public class MazeBot {
	

	static double maxSpeed = 200;
	static double errorTotal;
	static double lastError = 0;
	static double proportional;
	static double integral;
	static double derivative;
	static double targetDistance = 5; // cm
    static int defaultSpeed = 100;

    static double lastLeftDistance = 0;
    static double lastRightDistance = 0;
	
	//Constants
	static double kp = 7.3;
	static double ki = 1.5;
	static double kd = 0.2;
	
    private static int calculateSleep(double error){

        error = Math.abs(error);
        int sleep = 0;

        if(0 < error && error <= 10){
            sleep = 20;
        }
        else if(11 < error && error <= 30){
            sleep = 30;
        }
        else{
            sleep = 70;
        }
        return sleep;

    }

	
	public static void main(String[] args) {
		
		//Create a robot object to use and connect to it
        Robot myRobot = new Robot("dia-lego-a1");
       
        //The robot is made of components which are themselves objects.
        //Create references to them as useful shortcuts
        Motor leftMotor = myRobot.getLargeMotor(Motor.Port.B);
        Motor rightMotor = myRobot.getLargeMotor(Motor.Port.C);
        TouchSensor button = myRobot.getTouchSensor(Sensor.Port.S4);
        UltrasonicSensor sensorL = myRobot.getUltrasonicSensor(Sensor.Port.S3);
        UltrasonicSensor sensorR = myRobot.getUltrasonicSensor(Sensor.Port.S2);
        
        
            
        while(!(button.isTouched())) {
            double leftDistance = sensorL.getDistance() * 100;
            double rightDistance = sensorR.getDistance() * 100;

            if(leftDistance == Double.POSITIVE_INFINITY || leftDistance == Double.NEGATIVE_INFINITY){
                System.out.println("CAUGHT LEFT INFINITY ERROR");
                if(lastLeftDistance != Double.POSITIVE_INFINITY && lastLeftDistance != Double.NEGATIVE_INFINITY){
                    leftDistance = 0;
                }
                else{
                    leftDistance = lastLeftDistance;
                }
                
            }
            if(rightDistance == Double.POSITIVE_INFINITY || rightDistance == Double.NEGATIVE_INFINITY){
                System.out.println("CAUGHT RIGHT INFINITY ERROR");
                if(lastRightDistance != Double.POSITIVE_INFINITY && lastRightDistance != Double.NEGATIVE_INFINITY){
                    rightDistance = 0;
                }
                else{
                    rightDistance = lastRightDistance;
                }
            }
            double sensorTarget = (leftDistance + rightDistance )/2;
            double error = (leftDistance ) - sensorTarget; 
            
            

            if(error != 0){
                errorTotal += error;
            }
            else{
                errorTotal = 0;
            }


            if(error == 0){

                derivative = 0;
            }

            proportional = error * kp;
            integral = errorTotal * ki;
            derivative = (error - lastError) * kd;
            lastError = error;

            double speed = Math.abs((proportional + integral + derivative) * 10);
            

            System.out.println("Error: "+error);
            System.out.println("DISTANCE LEFT: "+ leftDistance);
            System.out.println("DISTANCE RIGHT: "+ rightDistance);
            System.out.println("SPEED: " + speed);

            if(speed >= maxSpeed){
                speed = maxSpeed;
            }
            double motorSpeed = speed;
            System.out.println("motor SPEED AFTER MAX:" + motorSpeed);
            
            


            if(error == 0){
                leftMotor.setSpeed(defaultSpeed);
                rightMotor.setSpeed(defaultSpeed);
                leftMotor.forward();
                rightMotor.forward();
                System.out.println("go");

            }
            int sleepTime = calculateSleep(error); 

            //Reverse right wheel
            if((leftDistance <= 3)&&(leftDistance > 0)){
                leftMotor.setSpeed(0);
                rightMotor.backward();
                myRobot.sleep(900);
                //leftMotor.setSpeed(defaultSpeed);
                // leftMotor.forward();
                // rightMotor.forward();
                System.out.println("Reverse Right ~~~~~~~~~~~~~~~~~~~~~~");
            }

            //Reverse left wheel
            if((rightDistance <= 3)&&(rightDistance > 0)){
                leftMotor.backward();
                rightMotor.setSpeed(0);
                myRobot.sleep(900);
                //rightMotor.setSpeed(defaultSpeed);
                // leftMotor.forward();
                // rightMotor.forward();
                System.out.println("Reverse Left ~~~~~~~~~~~~~~~~~~~~~~");
            }


            //Turn Left
            if(error > 0){
                myRobot.sleep(sleepTime);
                leftMotor.setSpeed(defaultSpeed);
                rightMotor.setSpeed((int)motorSpeed);
                leftMotor.forward();
                rightMotor.forward();
                System.out.println("leftTurn");
            }
            //Turn Right
            else{
                myRobot.sleep(sleepTime);
                rightMotor.setSpeed(defaultSpeed);
                leftMotor.setSpeed((int)motorSpeed);
                leftMotor.forward();
                rightMotor.forward();
                System.out.println("right Turn");

            }
            lastLeftDistance = leftDistance;
            lastRightDistance = rightDistance;

        
        }
    


		

		// //Disconnect from the Robot
        myRobot.close();
		

	}


}


 // // Error = measured distanced - actuall distance
// // proportional = error * kp

// // Every time through loop add errorTotal += error 
// // Intergral = errorTotal * ki

// // Derivate = rate of change of error 
// // Everytime through loop lastError = error (at botttom of loop)
 // // Derivative = error - lastError * kd

// //integralActive == set to value (Don't start accumalting error until above this threshold

// //Cap of integral term from growing too large with a limit value -> (50/ki)

// //Power sent to motor = proportional + Integral + Derivative

// //Have a wait time at the end of the loop
// import ShefRobot.*;
// import java.util.concurrent.TimeUnit;

// public class MazeBot {
	

	// static double maxSpeed = 200;
	// static double errorTotal;
	// static double lastError = 0;
	// static double proportional;
	// static double integral;
	// static double derivative;
	// static double targetDistance = 5; // cm
    // static int defaultSpeed = 100;

    // static double lastLeftDistance = 0;
    // static double lastRightDistance = 0;
	
	// //Constants
	// static double kp = 7.3;
	// static double ki = 1.5;
	// static double kd = 0.2;
	
    // private static int calculateSleep(double error){

        // error = Math.abs(error);
        // int sleep = 0;

        // if(0 < error && error <= 10){
            // sleep = 20;
        // }
        // else if(11 < error && error <= 30){
            // sleep = 30;
        // }
        // else{
            // sleep = 70;
        // }
        // return sleep;

    // }

	
	// public static void main(String[] args) {
		
		// //Create a robot object to use and connect to it
        // Robot myRobot = new Robot("dia-lego-a1");
       
        // //The robot is made of components which are themselves objects.
        // //Create references to them as useful shortcuts
        // Motor leftMotor = myRobot.getLargeMotor(Motor.Port.B);
        // Motor rightMotor = myRobot.getLargeMotor(Motor.Port.C);
        // TouchSensor button = myRobot.getTouchSensor(Sensor.Port.S4);
        // UltrasonicSensor sensorL = myRobot.getUltrasonicSensor(Sensor.Port.S3);
        // UltrasonicSensor sensorR = myRobot.getUltrasonicSensor(Sensor.Port.S2);
        
        
            
        // while(!(button.isTouched())) {
            // double leftDistance = sensorL.getDistance() * 100;
            // double rightDistance = sensorR.getDistance() * 100;

            // if(leftDistance == Double.POSITIVE_INFINITY || leftDistance == Double.NEGATIVE_INFINITY){
                // System.out.println("CAUGHT LEFT INFINITY ERROR");
                // if(lastLeftDistance != Double.POSITIVE_INFINITY && lastLeftDistance != Double.NEGATIVE_INFINITY){
                    // leftDistance = 0;
                // }
                // else{
                    // leftDistance = lastLeftDistance;
                // }
                
            // }
            // if(rightDistance == Double.POSITIVE_INFINITY || rightDistance == Double.NEGATIVE_INFINITY){
                // System.out.println("CAUGHT RIGHT INFINITY ERROR");
                // if(lastRightDistance != Double.POSITIVE_INFINITY && lastRightDistance != Double.NEGATIVE_INFINITY){
                    // rightDistance = 0;
                // }
                // else{
                    // rightDistance = lastRightDistance;
                // }
            // }
            // double sensorTarget = (leftDistance + rightDistance )/2;
            // double error = (leftDistance ) - sensorTarget; 
            
            

            // if(error != 0){
                // errorTotal += error;
            // }
            // else{
                // errorTotal = 0;
            // }


            // if(error == 0){

                // derivative = 0;
            // }

            // proportional = error * kp;
            // integral = errorTotal * ki;
            // derivative = (error - lastError) * kd;
            // lastError = error;

            // double speed = Math.abs((proportional + integral + derivative) * 10);
            

            // System.out.println("Error: "+error);
            // System.out.println("DISTANCE LEFT: "+ leftDistance);
            // System.out.println("DISTANCE RIGHT: "+ rightDistance);
            // System.out.println("SPEED: " + speed);

            // if(speed >= maxSpeed){
                // speed = maxSpeed;
            // }
            // double motorSpeed = speed;
            // System.out.println("motor SPEED AFTER MAX:" + motorSpeed);
            
            


            // if(error == 0){
                // leftMotor.setSpeed(defaultSpeed);
                // rightMotor.setSpeed(defaultSpeed);
                // leftMotor.forward();
                // rightMotor.forward();
                // System.out.println("go");

            // }
            // int sleepTime = calculateSleep(error); 

            // // //Reverse right wheel
            // // if(leftDistance <= 5){
                // // leftMotor.setSpeed(0);
                // // rightMotor.backward();
                // // myRobot.sleep(900);
                // // //leftMotor.setSpeed(defaultSpeed);
                // // // leftMotor.forward();
                // // // rightMotor.forward();
                // // System.out.println("Reverse Right ~~~~~~~~~~~~~~~~~~~~~~");
            // // }

            // // //Reverse left wheel
            // // if(rightDistance <= 5){
                // // leftMotor.backward();
                // // rightMotor.setSpeed(0);
                // // myRobot.sleep(900);
                // // //rightMotor.setSpeed(defaultSpeed);
                // // // leftMotor.forward();
                // // // rightMotor.forward();
                // // System.out.println("Reverse Left ~~~~~~~~~~~~~~~~~~~~~~");
            // // }


            // //Turn Left
            // if(error > 0){
                // myRobot.sleep(sleepTime);
                // leftMotor.setSpeed(defaultSpeed);
                // rightMotor.setSpeed((int)motorSpeed);
                // leftMotor.forward();
                // rightMotor.forward();
                // System.out.println("leftTurn");
            // }
            // //Turn Right
            // else{
                // myRobot.sleep(sleepTime);
                // rightMotor.setSpeed(defaultSpeed);
                // leftMotor.setSpeed((int)motorSpeed);
                // leftMotor.forward();
                // rightMotor.forward();
                // System.out.println("right Turn");

            // }
            // lastLeftDistance = leftDistance;
            // lastRightDistance = rightDistance;

        
        // }
    


		

		// // //Disconnect from the Robot
        // myRobot.close();
		

	// }


// }
