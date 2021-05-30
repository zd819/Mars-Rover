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


    private static int Control_Port = 8071; //Port used to open socket (must be port forward enabled)

    public Set<WebSocket> WebCon; //Websocket object
    public SynchronousQueue<String> Test;
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
    	}
    }
    @Override//If a message is recieved from the client, default echo back to client
    public void onMessage(WebSocket conn, String message) {
        System.out.println("Message from Control: " + message);
        for (WebSocket sock : WebCon) {
            sock.send(message);
            Writer(message);
        }
    }

    @Override//Exception caused by websocket errors
    public void onError(WebSocket conn, Exception ex) {
        //ex.printStackTrace();
        if (conn != null) {//If there is an error when creating websocket object??
            WebCon.remove(conn);
            // do some thing if required
        }
        System.out.println("ERROR from " + conn.getRemoteSocketAddress().getAddress().getHostAddress());
    }

    public void run() {
    	System.out.println("Runnable run method ran");
    	Test = Front.getControlQ();
    	while(true){

    	}
//    		Q call = new Q(methodName, args);
//    		Front.put(100);
//    		Object result = call.getResult();

    }
    public void Writer(String message) {

			try {
				Test.put(message);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
    	for (WebSocket sock : WebCon) {
        	//WebCon.send(message);
    		if(message !="0") {
    			System.out.println("From Control Thread : ");
    			sock.send(message);
    		}
    	}
    }

	@Override
	public void onStart() {
		System.out.println("Control Connection started");

	}


}
