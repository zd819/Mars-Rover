package Full;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.net.InetSocketAddress;
//import java.net.http.WebSocket;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.SynchronousQueue;

//import org.slf4j.*;
import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;
import java.util.HashMap;

public class CommandThread extends WebSocketServer implements Runnable{

    private static int Command_Port = 8080; //Port used to open socket (must be port forward enabled)

    private static Set<WebSocket> WebCom; //Websocket object
    public volatile static SynchronousQueue<String> ControlQ;
    public volatile static SynchronousQueue<String> CommandQ;
    public static HashMap<String, String> Coordinates = new HashMap<String, String>(); //Hashmap for co-ordinates of obstacles

    	public CommandThread(SynchronousQueue<String> ControlQ, SynchronousQueue<String>CommandQ) {
	        super(new InetSocketAddress(Command_Port)); //Open socket on port designated
    		CommandThread.ControlQ = ControlQ;
    		CommandThread.CommandQ = CommandQ;
	        WebCom = new HashSet<>();//Buffer
	        System.out.println("Command socket initiated");
	    }

    @Override//If websocket connection handshake is successful to port, from a Client
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
    	System.out.println("Before Command connection verified");
        WebCom.add(conn);
        System.out.println("Command has connected with ip :  " + conn.getRemoteSocketAddress().getAddress().getHostAddress());
    }

    @Override//If Websocket connection is disconnected from client
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {
    	if(conn != null) {
    	WebCom.remove(conn);
        System.out.println("Closed connection to command :  " + conn.getRemoteSocketAddress().getAddress().getHostAddress());
    	}
    }
    @Override//If a message is recieved from the client, default echo back to client
    public void onMessage(WebSocket conn, String message) {
        System.out.println("Message from Command: " + message);
        if(!message.equals("Connected")) {//!= compares memory addresses in java so we can't use this
	        Writer(message);//Write to the command thread
	        //this.Listener();
    	}
        for (WebSocket sock : WebCom) {
        	//WebCon.send(message);
            sock.send(message);
            //sq.put(100);//Testing synchronous queues
        }
    }

    @Override//Exception caused by websocket errors
    public void onError(WebSocket conn, Exception ex) {
        //ex.printStackTrace();
        if (conn != null) {//If there is an error when creating websocket object??
            WebCom.remove(conn);
            // do some thing if required
        }
        System.out.println("ERROR from " + conn.getRemoteSocketAddress().getAddress().getHostAddress());
    }

     //Try to read from Synchronous queue
	 //If no value in synchronous queue then
	 //Value not needed to be piped through to command
	 //Yet and so we throw an exception and do nothing
    public static void Listener() {
    	Thread listenThread = new Thread() {
    		//String CommandData; //Formatted data we send to the Command App
    		public void run() {
    			System.out.println("Command Listener running");
    			while(true) {
    				System.out.println(ControlQ.isEmpty());
    				try {
						Thread.sleep(1000);
					} catch (InterruptedException e1) {
						// TODO Auto-generated catch block
						e1.printStackTrace();
					}
	    			//System.out.println(ControlOut.remainingCapacity());
    				if(ControlQ.isEmpty() != false ) {//Thought isEmpty returns true if empty
    												 //Seems like it returns true if has a value
		    			String message;
		    			String cords;
		    			int index;
		    			try {
							message = ControlQ.take();
							System.out.println("-----Taken from Control Queue is :"  + message + "-----");
							cords = Map(message);
							index = permData(message);
							System.out.println("-----EXTRA CORDINATES----- : " + cords);
							message = message.substring(0,index) + cords + "/";
							System.out.println("-----FINAL MESSAGE----- : " + message);
							for (WebSocket sock : WebCom) {
								sock.send(message);
					            System.out.println("MESSAGE SENT BACK TO COMMAND, SUCCESS");
					        }

		    			}catch(InterruptedException e){
		    				System.out.println("-----Command Not Connected-----");
		    			} catch (IOException e) {
							// Error with Writing to file, check file name
		    				System.out.println("-----Error with Writing to file, check file name-----");
		    				e.printStackTrace();
						}
	    			}
    			}
    		}
    	}; listenThread.start();

    }
    public static int permData(String message) {
    	int x = 0;
    	int y = 0;
    	for(int i = 0; i < 4; i++) {
    		x = message.indexOf("]");
    		message = message.substring(x+1);
    		y += x + 1;
    	}
    	//System.out.println("-----INDEX OF CORDINATES----- : " + y);

    	return y;
    }

    public static String Map(String message) throws IOException {
    	String newCords = "";
    	message = message.substring(2);
    	FileWriter writer = new FileWriter("Database.txt", true);
        BufferedWriter bufferedWriter = new BufferedWriter(writer);

    	for(int i = 0; i < 4; i++) {
    		int x = message.indexOf("]");
    		message = message.substring(x+1);
    		System.out.println("STRINGED MESSAGE: " + message);
    		if(i==2) {
    			bufferedWriter.write(message);
                bufferedWriter.close();
    		}
    		if(i==3) {
    			bufferedWriter.write("," + message);
    			bufferedWriter.newLine();
                bufferedWriter.close();
    		}

    	}
    	for(int i = message.indexOf("]") ;i< message.length(); message = message.substring(i+1), i = message.indexOf("]", message.indexOf("]") +1) ){
    		if(Coordinates.containsKey(message.substring(0,i+1)) == false) {
    			//Store data in text file database
    			bufferedWriter.write(message.substring(0,i+1) + " : ");
                bufferedWriter.write( message.substring(i, message.indexOf("]", message.indexOf("]") +1)));
                bufferedWriter.newLine();

                bufferedWriter.close();

    			newCords += message.indexOf("]", message.indexOf("]") +1);
    			Coordinates.put(message.substring(0,i+1), message.substring(i, message.indexOf("]", message.indexOf("]") +1) ));
    			System.out.println("New Co-ordinate : " + message.substring(0,i+1));
    		}
    		if(i==-1) {
    			System.out.println("RETURNING");
    	    	return newCords;
    		}
    	}
    	//Now message in form color1][x1,y1][color2][x2,y2][color3][x3,y3]/...
    	//Way to use char array strings
    	//String[] Obstacles = new String[10];
    	//Obstacles[0] = message.substring(0,3);
    	System.out.println("RETURNING");
    	return newCords;
    }

    public void Writer(String message) {
		try {
			System.out.println("Trying to put on Command Queue : " + message);
			CommandQ.put(message);
			//System.out.println("Message put on Command Queue");
		} catch (InterruptedException e) {
			System.out.println("Unable to put message on Command Queue");
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
    }

	@Override
	public void onStart() {
		System.out.println("Command Thread started");
//		ControlOut = Front.getControlQ();
//		CommandOut = Front.getCommandQ();
		Listener();

	}
}
