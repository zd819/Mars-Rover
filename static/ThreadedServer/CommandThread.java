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

public class CommandThread extends WebSocketServer implements Runnable{



    private static int Command_Port = 8076; //Port used to open socket (must be port forward enabled)

    private Set<WebSocket> WebCom; //Websocket object
	public SynchronousQueue<String> Test;
    	public CommandThread() {
	        super(new InetSocketAddress(Command_Port)); //Open socket on port designated
	        WebCom = new HashSet<>();//Buffer
	        System.out.println("Command socket initiated");
	    }

    @Override//If websocket connection handshake is successful to port, from a Client
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
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

    public void run() {
    	System.out.println("Runnable run method ran");
    	Test = Front.getControlQ();
    	while(true){
    		Listener();
    	}
//    		Q call = new Q(methodName, args);
//    		Front.put(100);
//    		Object result = call.getResult();

    }

     //Try to read from Synchronous queue
	 //If no value in synchronous queue then
	 //Value not needed to be piped through to command
	 //Yet and so we throw an exception and do nothing
    public void Listener() {
    	String message = "0";
    	try {
			message = Test.take();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
    	for (WebSocket sock : WebCom) {
        	//WebCon.send(message);
    		if(message !="0") {
    			System.out.println("From Control Thread : ");
    			sock.send(message);
    		}
    	}
    }
	@Override
	public void onStart() {
		System.out.println("Command Connection started");
//		/Transmission();
	}


}
