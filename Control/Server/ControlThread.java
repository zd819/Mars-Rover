package Full;

import java.net.InetSocketAddress;
//import java.net.http.WebSocket;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.SynchronousQueue;

//import org.slf4j.*;
import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

public class ControlThread extends WebSocketServer implements Runnable{

    private static int Control_Port = 8087; //Port used to open socket (must be port forward enabled)

    public Set<WebSocket> WebCon; //Websocket object
    public volatile static SynchronousQueue<String> ControlQ;
    public volatile static SynchronousQueue<String> CommandQ;

	    public ControlThread(SynchronousQueue<String> ControlQ, SynchronousQueue<String>CommandQ) {
	        super(new InetSocketAddress(Control_Port)); //Open socket on port designated
	        ControlThread.ControlQ = ControlQ;
    		ControlThread.CommandQ = CommandQ;
	        WebCon = new HashSet<>();//Buffer
	        System.out.println("Control socket initiated");
	    }



    @Override//If websocket connection handshake is successful to port, from a Client
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
    	WebCon.add(conn);
        System.out.println("Control has connected with ip :  " + conn.getRemoteSocketAddress().getAddress().getHostAddress());

    }

    @Override//If Websocket connection is disconnected from client
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {
    	if(conn != null) {
    	WebCon.remove(conn);
        System.out.println("Closed connection to Control :  " + conn.getRemoteSocketAddress().getAddress().getHostAddress());
    	//conn.close();
    	}
    }
    @Override//If a message is recieved from the client, default echo back to client
    public void onMessage(WebSocket conn, String message) {
    	System.out.println("Message from Control : " + message);
    	if(!message.equals("Connected")) {//!= compares memory addresses in java
	        Writer(message);//Write to the command thread
    	}
    }

    @Override//Exception caused by websocket errors
    public void onError(WebSocket conn, Exception ex) {
        //ex.printStackTrace();
        if (conn != null) {//If there is an error when creating websocket object??
            WebCon.remove(conn);
            System.out.println("No error");
        }
        System.out.println("ERROR from " + conn.getRemoteSocketAddress().getAddress().getHostAddress());
    }

//    public void run() {
//    	//Not using runnable method run, since the thread will ONLY RUN this method and not
//    	//continue to run the control class on the thread, or thats what seemed to happen during testing
//    }

	public void Listener() {
    	//Since Synchronous Queues are Blocking, another thread hasn't taken
		//Once this thread puts an item on the queue,
    	//This function will Block all running methods/computation and wait
    	//For there to be an item in the Synchronous Queue
    	//We utilise the property of put/take, as when you take you remove from the
    	//item from the synchronous queue, so we check the number of items in the queue
    	//Until there is a block (no items) and if so, we don't send anything to the Control
    	//ESP32 unit since there is no data from the Command Thread (via the command app)
    	//To send, since the Synchronous queue hasnt been updated
		Thread listenThread = new Thread() {
    		public void run() {
    			System.out.println("Control Listener running");
    			while(true) {
    				System.out.println(CommandQ.isEmpty());
    				try {
						Thread.sleep(1000);
					} catch (InterruptedException e1) {
						// TODO Auto-generated catch block
						e1.printStackTrace();
					}
	    			//System.out.println(ControlOut.remainingCapacity());
    				if(CommandQ.isEmpty() != false ) {//Thought isEmpty returns true if empty
    												 //Seems like it returns true if has a value
		    			String message;
		    			try {
							message = CommandQ.take();
							System.out.println("-----Taken from Command Queue is : " + message + "-----");
							for (WebSocket sock : WebCon) {
					        	//WebCon.send(message);
					            sock.send(message);
					            System.out.println("MESSAGE SENT BACK TO CONTROL, SUCCESS");
					        }

		    			}catch(InterruptedException e){
		    				System.out.println("-----Control Not Connected-----");
		    			}
	    			}
    			}
    		}


    	}; listenThread.start();
    }

    public void Writer(String message) {
			try {
				//System.out.println(ControlQ.remainingCapacity() + " Trying to put on Control Queue : " + message);
				ControlQ.put(message);
				//System.out.println("-----Capacity-----" + ControlQ.remainingCapacity());
				CommandThread.Listener();
				//System.out.println("Message put on Control Queue");
			} catch (InterruptedException e) {
				System.out.println("Unable to put message on Control Queue");
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
    }



	@Override
	public void onStart() {
		System.out.println("Control Thread started");
		Listener();
	}

}
