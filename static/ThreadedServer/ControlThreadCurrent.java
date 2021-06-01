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

    private static int Control_Port = 8080; //Port used to open socket (must be port forward enabled)

    public Set<WebSocket> WebCon; //Websocket object
    public SynchronousQueue<String> ControlOut;
    public SynchronousQueue<String> CommandOut;

	    public ControlThread() {
	        super(new InetSocketAddress(Control_Port)); //Open socket on port designated
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
        Writer(message);//Write to the command thread
        //this.Listener();
        for (WebSocket sock : WebCon) {
            sock.send(message);
            System.out.println("Sent message back to Control Unit");
            //Writer(message);
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
//    	//continue to run the control class on the thread
//    }

	public void Listener() {
    	System.out.println("-----1-----");
    	String message = "0";
    	System.out.println("-----2-----");
    	//Since Synchronous Queues are Blocking, if there is no item to take
    	//This function will Block all running methods/computation and wait
    	//For there to be an item in the Synchronous Queue
    	//We utilise the property of put/take, as when you take you remove from the
    	//item from the synchronous queue, so we check the number of items in the queue
    	//Until there is a block (no items) and if so, we don't send anything to the Control
    	//ESP32 unit since there is no data from the Command Thread (via the command app)
    	//To send, since the Synchronous queue hasnt been updated
    	for(int i = 0; i<5; i++){
    	//while(true) {
	    	if(CommandOut.remainingCapacity() != 0) {
		    	try {
					message = CommandOut.take();
					if(CommandOut.take()==null) {
						System.out.println("-----10-----");
					}
					//System.out.println("Message taken from Command Queue");
					System.out.println("-----4-----");
					for (WebSocket sock : WebCon) {
			    		System.out.println("Writing to ESP32");
			        	//WebCon.send(message);
			    		if(message !="0") {
			    			System.out.println("From Command Thread : ");
			    			sock.send(message);
			    		}
			    		System.out.println("-----5-----");
			    	}
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					throw new ArithmeticException("Catch Executed");
					//System.out.println("Error taking from Control Q");
					//e.printStackTrace();
				}finally {
					System.out.println("Listener Try executed");
				}
	    	}
	    	System.out.println("-----Capacity-----" + CommandOut.remainingCapacity());

    	}
    }

    public void Writer(String message) {
			try {
				System.out.println("Trying to put on Control Queue : " + message);
				ControlOut.put(message);
				System.out.println("Message put on Control Queue");
			} catch (InterruptedException e) {
				System.out.println("Unable to put message on Control Queue");
				// TODO Auto-generated catch block
				e.printStackTrace();
			}finally {
				System.out.println("Writer Try executed");
			}
    }

    public void TestFunction() {
    	System.out.println("Function call in start working");
    }

	@Override
	public void onStart() {
		System.out.println("Control Thread started");
		ControlOut = Front.getControlQ();
		CommandOut = Front.getCommandQ();
		Listener();
	}

}
