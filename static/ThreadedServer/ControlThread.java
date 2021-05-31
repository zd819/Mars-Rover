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
    public SynchronousQueue<String> CommandQ;
    public SynchronousQueue<String> ControlQ;
	    public ControlThread() {
	        super(new InetSocketAddress(Control_Port)); //Open socket on port designated
	        WebCon = new HashSet<>();//Buffer
	        System.out.println("Control socket initiated");
	    }



    @Override//If websocket connection handshake is successful to port, from a Client
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
    	System.out.println("Before Control connection verified");
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
        for (WebSocket sock : WebCon) {
        	System.out.println("Test1");
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
    	System.out.println("Listener running");
    	String message = "0";
    	while(true) {
    		System.out.println("Listening to Command Thread");
	    	try {
				message = CommandQ.take();
				System.out.println("Message taken from Command Queue");
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
	    	for (WebSocket sock : WebCon) {
	        	//WebCon.send(message);
	    		if(message !="0") {
	    			System.out.println("From Command Thread : ");
	    			sock.send(message);
	    		}
	    	}
    	}
    }

    public void Writer(String message) {
    	System.out.println("Writer called");
			try {
				System.out.println("Trying to put on Command Queue");
				ControlQ.put(message);
				System.out.println("Message put on Command Queue");
			} catch (InterruptedException e) {
				System.out.println("Unable to put message on Command Queue");
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
    }

	@Override
	public void onStart() {
		System.out.println("Control Thread started");
		ControlQ = Front.getControlQ();
		CommandQ = Front.getCommandQ();
		System.out.println("Retrieved Control Thread");
		Listener();
	}

}
