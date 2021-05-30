package Full;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Scanner;
import java.util.concurrent.SynchronousQueue;
//import org.slf4j.*;
import java.io.IOException;
import java.net.ServerSocket;


public class Front {

	public SynchronousQueue<Q> CommandQueue = new SynchronousQueue<Q>();
    public SynchronousQueue<Q> ControlQueue= new SynchronousQueue<Q>();

    public static SynchronousQueue<String> CommandQ = new SynchronousQueue<String>();
    public static SynchronousQueue<String> ControlQ = new SynchronousQueue<String>();

    public static void main(String[] args) throws IOException {
    	SynchronousQueue<Integer> CommandQueueTest = new SynchronousQueue<Integer>();
	    	Thread Command = new Thread(); {
	    		new CommandThread().start();
	    	};
	    	Command.start();

	    	Thread Control= new Thread(); {
	    		new ControlThread().start();
	    	};
	    	Control.start();

//	        try (new WebsocketServer().start()) {
//	        } catch (IOException e) {
//	            System.out.println("Could not create WebSocket Server");
//	            System.exit(-1);
//	        }
//
//	    }

	    }

    public static SynchronousQueue<String> getCommandQ() {
		return CommandQ;
    }
    public static SynchronousQueue<String> getControlQ() {
		return ControlQ;
    }


}
