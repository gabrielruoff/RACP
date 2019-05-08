import java.io.BufferedReader;
import java.io.IOException;
import java.io.OutputStream;
import java.text.DecimalFormat;
import java.util.Scanner;

import com.leapmotion.leap.Controller;

import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;

public class programmedMove implements SerialPortEventListener {


	
	static OutputStream out = null;
	static BufferedReader input;
	
	static DecimalFormat df = new DecimalFormat("000");
	
	//Define joints and set position to 90
	static Joint zRot = new Joint(90);
	static Joint shoulder = new Joint(90);
	static Joint elbow = new Joint(90);
	static Joint wrist = new Joint(90);
	static Joint clawRot = new Joint(90);
	static Joint claw = new Joint(90);
	
	public static void main(String[] args) {
		
		//Connect to COM port
		try
		{
			//Device
		(new programmedMove()).connect("/dev/cu.usbserial-1410");
	
		Thread.sleep(3000);
		
		}
		catch ( Exception e )
		{
		e.printStackTrace();
		}

		//First move
		shoulder.setPosition(50);
		elbow.setPosition(120);
		writeData();
		
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		//Second move
		claw.setPosition(0);
		clawRot.setPosition(90);
		writeData();
		
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		//Third move
		wrist.setPosition(10);
		writeData();
		
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		//Fourth move
		claw.setPosition(90);
		writeData();
		
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		//Fifth move
		shoulder.setPosition(90);
		writeData();
		
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		//Sixth move
		wrist.setPosition(180);
		writeData();
		
	}
	
	static void writeData() {
		
		String data = df.format(zRot.position) + ":" + df.format(shoulder.position) + ":" +  df.format(elbow.position) + ":" + df.format(wrist.position) + ":" + df.format(clawRot.position) + ":" + df.format(claw.position);
		
		writeToArduino(data);
		
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
