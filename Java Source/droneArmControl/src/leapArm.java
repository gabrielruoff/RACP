import java.io.BufferedReader;
import java.io.IOException;
import java.io.OutputStream;
import java.text.DecimalFormat;

import com.leapmotion.leap.Arm;
import com.leapmotion.leap.Bone;
import com.leapmotion.leap.Controller;
import com.leapmotion.leap.Finger;
import com.leapmotion.leap.FingerList;
import com.leapmotion.leap.Frame;
import com.leapmotion.leap.Gesture;
import com.leapmotion.leap.Hand;
import com.leapmotion.leap.HandList;
import com.leapmotion.leap.Listener;
import com.leapmotion.leap.Vector;

import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;

class SampleListenerMain extends Listener {

	//define position lock booleans
	public boolean leftLock = false;
	public boolean rightLock = false;
	
	public static int oldZRotationPos;
	public static int oldShoulderAngle;
	public static int oldElbowAngle;
	public static int oldWristAngle;
	public static int oldClawPos;
	public static int oldClawRotPos;
	public static double MAX_INSTANTANEOUS_VELOCITY = 50.00; //max degrees/sec to stabilize data
	
	public static double oldZRotationApprox, oldShoulderApprox, oldElbowApprox, oldWristApprox, oldClawPosApprox = 0;
	
	public static double gestureIndex = 0;
	
	//initialize Kalman Filters
	public static KalmanFilter shoulderKFilter = new KalmanFilter();
	public static KalmanFilter elbowKFilter = new KalmanFilter();
	
	String data = "";
	String oldLeft = "";
	String oldRight = "";
	static DecimalFormat df = new DecimalFormat("000");
	
	//Displacement variables
	double deltaX, deltaY, deltaZ, angle;
	
public void onInit(Controller controller) {
//System.out.println("Initialized");

}

public void onConnect(Controller controller) {
//System.out.println("Connected");
controller.enableGesture(Gesture.Type.TYPE_CIRCLE);
controller.enableGesture(Gesture.Type.TYPE_KEY_TAP);
controller.config().setFloat("Gesture.KeyTap.MinDownVelocity", 40.0f);
controller.config().setFloat("Gesture.KeyTap.HistorySeconds", .2f);
controller.config().setFloat("Gesture.KeyTap.MinDistance", 1.0f);
controller.config().save();

}

public void onDisconnect(Controller controller) {
//System.out.println("Disconnected");
}

public void onExit(Controller controller) {
//System.out.println("Exited");
}

public void onFrame(Controller controller) {
	
	long startTime = System.nanoTime();
	
	data = "";
	
	//Define position variables
	double shoulderAngle, elbowAngle, clawPos, zRotationPos, wristAngle, clawRotPos;
	
	//Define object variables
	//Frame
	Frame frame = controller.frame();
	//Hands
	Hand leftHand = frame.hands().leftmost();
	Hand rightHand = frame.hands().rightmost();
	//Arms
	Arm leftArm = leftHand.arm();
	Arm rightArm = rightHand.arm();
	//Fingers
	FingerList fingerList = rightHand.fingers().fingerType(Finger.Type.TYPE_INDEX);
	Finger rightIndexFinger = fingerList.get(0); 
	
	//Right Thumb
	fingerList = rightHand.fingers().fingerType(Finger.Type.TYPE_THUMB);
	Finger rightThumb = fingerList.get(0);
	
	//find the distance between the bones to detect pinch
	Vector rightIndexDistal = rightIndexFinger.bone(Bone.Type.TYPE_DISTAL).center();
	Vector rightThumbDistal = rightThumb.bone(Bone.Type.TYPE_DISTAL).center();
	
	/* Control of robotic arm with Z-rotation based on the left hand, arm 'wrist' position based on the wrist,
	 * arm 'elbow position based on the elbow, and claw based on the fingers. 'Shoulder' is based on the left elbow
	 */
	
	//Control position locks for left hand controls and right hand controls
	//Gesture gesture = new Gesture(gesture);
	/*for(Gesture gesture : frame.gestures()) {
		
		
		HandList handsForGesture = gesture.hands();
		
		//Sleep for 20ms to let gesture be recognized
		try {
			Thread.sleep(1);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		
		//print handList
		for(Hand hand : handsForGesture) {
			
			if(hand.equals(rightHand)) {
				
				System.out.println("Right Hand");
				
			} else if(hand.equals(leftHand)) {
				
				System.out.println("Left Hand");
				
			}
			
		}
		//sleep
		try {
			Thread.sleep(100);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		
		switch(gesture.type()) {
		
		case TYPE_KEY_TAP:
			
			System.out.println("Key tap from" + handsForGesture + " Hand");
			
			for(Hand hand : handsForGesture) {
				
				if(hand.equals(rightHand)) {
					
					rightLock = !rightLock;
					
				} else if(hand.equals(leftHand)) {
					
					leftLock = !leftLock;
					
				}
				
			}
			
			break;
			
		default:
			
			System.out.println("Unrecognized gesture");
			
			break;
		
		}
		
		break;
		
	}*/
	
	//Lock left controls if left lock is on
	if(!leftLock) {
	
		//'Shoulder' control. MIRRORS LEFT ELBOW ANGLE
		//find angle between the left elbow and the left wrist center
		Vector leftElbow = leftArm.elbowPosition();
		Vector leftWrist = leftArm.wristPosition();
		
		deltaZ = leftElbow.getZ() - leftWrist.getZ();
		deltaY = leftElbow.getY() - leftWrist.getY();
		
		angle = Math.atan(deltaY/deltaZ);
		
		//map angle so servo can understand it
		shoulderAngle = leapArm.map(angle, (0.1), (-0.8), 0, 180);
		
		//Set Kalman Filter sensor input to angle
		if(shoulderAngle<=0 || Double.isNaN(shoulderAngle)) {
			
		shoulderKFilter.Zk.setEntry(0, 0, 0);
			
		} else {
	
		System.out.println("Yk_prev: " + shoulderKFilter.Yk_prev);
		shoulderKFilter.Zk.setEntry(0, 0, shoulderAngle);
		
		}
		
		//Run Kalman Filter
		shoulderAngle = leapArm.filterInput(shoulderKFilter);
		
		//Write position to 'shoulder'
		
		//Z-rotation control
		Vector leftHandPos = leftHand.palmPosition();
		//rotate z-axis with speed proportional to left hand X position
		//map X position to motor power
		//bounded 50/230 so hands don't knock into each other
		zRotationPos = leapArm.map(leftHandPos.getX(), -230, 50, 0, 180);
		oldZRotationPos = (int) zRotationPos;
		////System.out.println("zRotationPos: " + zRotationPos);
		data += df.format(zRotationPos);
		
		//write shoulder value to data
		if(Double.isNaN(shoulderAngle) || shoulderAngle <= 0) {
			data += ":000";
		} else {
		data+=":" + df.format(shoulderAngle);
		}
	
	//'elbow' control. MIRRORS LEFT WRIST ANGLE
	//find angle between the left wrist and palm
	leftWrist = leftArm.wristPosition();
	
	//refresh deltas and angle
	deltaZ = leftWrist.getZ() - leftHand.palmPosition().getZ();
	deltaY = leftWrist.getY() - leftHand.palmPosition().getY();
	
	angle = Math.atan(deltaY/deltaZ);
	
	//map angle so the servo can understand it
	elbowAngle = leapArm.map(angle, (-1), 0.5, 0, 180);
	
	//Set Kalman Filter sensor input to angle
	if(elbowAngle<=0 || Double.isNaN(elbowAngle)) {
		
	elbowKFilter.Zk.setEntry(0, 0, 0);
		
	} else {

	//System.out.println("Yk_prev: " + elbowKFilter.Yk_prev);
	elbowKFilter.Zk.setEntry(0, 0, elbowAngle);
	
	}
	
	//Run Kalman Filter
	elbowAngle = leapArm.filterInput(elbowKFilter);
	
	//Write elbow value to data
	//data += leapArm.linearApproximation(elbowAngle, oldElbowAngle, 2);
	//System.out.println("Returned: " + data);
	if(Double.isNaN(elbowAngle) || elbowAngle <= 0) {
		data += ":000";
	} else {
	data+=":" + df.format(elbowAngle);
	}
	
	//save the left side of the data string to send later if lock is turned on
	oldLeft = data;
	
	//Send the old value if left lock is on
	} else {
	
		data += oldLeft;
	
	}
	
	//'wrist' control
	//update vectors
	Vector rightElbow = rightArm.elbowPosition();
	Vector rightWrist = rightArm.wristPosition();
	
	Vector rightHandPos = rightHand.palmPosition();
	
	//update deltas
	deltaZ = rightWrist.getZ() - rightHandPos.getZ();
	deltaY = rightWrist.getY() - rightHandPos.getY();
	
	
	////System.out.println("Wrist pos: " + rightWrist.getX() + ", " + rightWrist.getY() + ", " + rightWrist.getZ());
	////System.out.println("Right hand pos: " + rightHandPos.getX() + ", " + rightHandPos.getY() + ", " + rightHandPos.getZ());
	
	angle = Math.atan(deltaY/deltaZ);
	
	////System.out.println("Wrist angle: " + angle);
	
	//map value
	wristAngle = leapArm.map(angle, 0.4, (-1.2), 0, 180);
	//Write wrist value to data
	//data+=leapArm.linearApproximation(wristAngle, oldWristAngle, 3);
	if(Double.isNaN(wristAngle) || wristAngle <= 0) {
		data += ":000";
	} else {
	data+=":" + df.format(wristAngle);
	}
	
	//Claw rotation position
	//bounded 50/230 so hands don't knock into each other
	////System.out.println("Right hand x: " + rightHandPos.getX());
	clawRotPos = leapArm.map(rightHandPos.getX(), 80, 170, 0, 180);
	//Write clawrot value to data
	//data+=leapArm.linearApproximation(clawRotPos, oldClawRotPos, 4);
	if(Double.isNaN(clawRotPos) || clawRotPos <= 0) {
		data += ":000";
	} else {
	data+=":" + df.format(clawRotPos);
	}
	
	
	
	//System.out.println("wristAngle: " + wristAngle + " degrees");
	
	//pinch control
	
	//Calculate distance between joints
	double distalDistance = Math.sqrt(Math.pow((rightIndexDistal.getX()-rightThumbDistal.getX()),2) + Math.pow((rightIndexDistal.getY()-rightThumbDistal.getY()),2) + Math.pow((rightIndexDistal.getZ()-rightThumbDistal.getZ()),2));
	////System.out.println("Distal Distance: " + distalDistance);
	if(distalDistance >= 50) {
		
		clawPos = 0;
		
	} else {
		
		clawPos = 90;
		
	}
	
	oldClawPos = (int) clawPos;

	data += ":" + df.format(clawPos);
		
	//System.out.println("ClawPos: " + clawPos);
	
	/* Write data to arduino
	 * FORMAT: z-rotation:shoulderPos:elbowAngle:wristAngle:clawRot:clawPos
	 */
	
	//Print data to terminal
	System.out.println("Data: " + data);
	System.out.println("Locks: " + leftLock + ", " + rightLock);
	
	/* wait for Arduino to catch up to avoid buffer overflow. Arduino can handle ~30 packets/sec (19bytes each) **need to update for new packet length
	 * basically see how long the Arduino takes to process one packet and flush the receiving arrays to prevent array 'pollution'.
	 */
	try {
		Thread.sleep(50);
	} catch (InterruptedException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}

	//send data to the arduino
	leapArm.writeToArduino(data);
	//System.out.println("Sent");
	
	System.out.println("This loop took " + (System.nanoTime()-startTime)/(Math.pow(10,6)) + "milliseconds");
	
	}


}

public class leapArm implements SerialPortEventListener {

	public static double map(double input, double in_min, double in_max, double out_min, double out_max) {
		
		return ((input - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
		
	}
	
	public static int filterInput(KalmanFilter k) {
		
		k.predict(k.Yk, k.A, k.Yk_prev, k.Pk, k.Pk_prev, k.Q);
		k.update(k.K, k.Pk, k.Pk_prev, k.H, k.R, k.YkHat, k.YkHat_prev, k.Zk, k.I);
		
		return (int) k.getOutput();
		
	}
	
	static OutputStream out = null;
	static BufferedReader input;
	
	public static void main(String[] args) {
		
		//Connect to COM port
		try
		{
			//Device
		(new leapArm()).connect("/dev/cu.usbmodem14101");
	
		Thread.sleep(3000);
		
		}
		catch ( Exception e )
		{
		e.printStackTrace();
		//System.exit(0);
		}
		
		// Create a sample listener and controller
		SampleListenerMain listener = new SampleListenerMain();
		Controller controller = new Controller();

		// Have the sample listener receive events from the controller
		controller.addListener(listener);
		// Keep this process running until Enter is pressed
		
		//System.out.println("Press Enter to quit...");
		try {
		System.in.read();
		} catch (IOException e) {
		e.printStackTrace();
		}

		// Remove the sample listener when done
		controller.removeListener(listener);
	}
	
	void connect ( String portName ) throws Exception {

		CommPortIdentifier portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
		if ( portIdentifier.isCurrentlyOwned() )
		{
		//System.out.println("Error: Port is currently in use");
		}
		else
		{
		CommPort commPort = portIdentifier.open(this.getClass().getName(),2000);

		if ( commPort instanceof SerialPort )
		{
		SerialPort serialPort = (SerialPort) commPort;
		serialPort.setSerialPortParams(4800,SerialPort.DATABITS_8,SerialPort.STOPBITS_1,
		SerialPort.PARITY_NONE);
		out = serialPort.getOutputStream();
		//input = new BufferedReader(new InputStreamReader(serialPort.getInputStream()));
		
		// add event listeners
		try {
					serialPort.addEventListener(this);
					serialPort.notifyOnDataAvailable(true);
				} catch (Exception e) {
					//System.err.println(e.toString());
		
		}
		}
		else
		{
		//System.out.println("Selected port is not a Serial Port");
		}
		}
		
		}

		public static void writeToArduino(String data)
		{
		String tmpStr = data;
		byte bytes[] = tmpStr.getBytes();
		try {
			/*//System.out.println("Sending Bytes: ");
			for(int i = 0; i<bytes.length; i++) {
				//System.out.println(bytes[i]);
			}*/
		out.write(bytes);
		} catch (IOException e) { }
		}
		
		public synchronized void serialEvent(SerialPortEvent oEvent) {
			if (oEvent.getEventType() == SerialPortEvent.DATA_AVAILABLE) {
				try {
					String inputLine=input.readLine();
					System.out.println("Received: " + inputLine);
				} catch (Exception e) {
					System.err.println(e.toString());
				}
			}
			// Ignore all the other eventTypes, but you should consider the other ones.
		}
	
}
