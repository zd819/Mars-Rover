package Full;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.HashSet;
import java.util.Scanner;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.SynchronousQueue;
//import org.slf4j.*;
import java.io.IOException;
import java.net.ServerSocket;


public class Front {

	//public SynchronousQueue<Q> CommandQueue = new SynchronousQueue<Q>();
    //public SynchronousQueue<Q> ControlQueue= new SynchronousQueue<Q>();

    public volatile static SynchronousQueue<String> CommandQ = new SynchronousQueue<String>(true);
    public volatile static SynchronousQueue<String> ControlQ = new SynchronousQueue<String>(true);

    public static void main(String[] args) throws IOException {

    	System.out.println(""
    			+ "                                                                    ||\r\n"
    			+ "                                                  __..--\".          ||\r\n"
    			+ "                                 __..--\"\"`._..--\"\" . . . .`.        ||\r\n"
    			+ "                         __..--\"\". . . . . . .`. . . . . . .`.      ||\r\n"
    			+ "                 __..--\"\". . . . .`. . . . . . .`. . . . . . .`.   //\r\n"
    			+ "         __..--\"\". . `.  . . . . . .`. . . . . . .`. . . . . . .`.//\r\n"
    			+ "  _..--\"\"  . . . . . . `.  . . . . . .`. . . . . . .`. . . . . . .||\r\n"
    			+ ":\". . . .`.  . . . . . . `.  . . . . . .`. . . . . . .`. . . . . .||`.\r\n"
    			+ "`:. . . . .`.  . . . . . . `.  . . . . . .`. . . . . . .`. . . . .||__>\r\n"
    			+ "  `:. . . . .`.  . . . . . . `.  . . . . . .`. . . . . . .`.__..-o||\r\n"
    			+ "    `:. . . . .`.  . . . . . . `.  . . . . . .`. . . . .`;Y\"->.  \"\"\r\n"
    			+ "      `:. . . . .`.  . . . . . . `.  . . . . . .`. . . __.>.:'\r\n"
    			+ "        `:. . . . .`.  . . . . . . `.  . . . . __..--\"\" ..+\"`.\r\n"
    			+ "   _..-._ `:. . . . .`.  . . . . . . `.__..--\"\" ....:::::.|   `.\r\n"
    			+ " .\"`` \\_--\" >:. . . . .`.  . . __..,-|\" . ..::::::::::::::`--\"\"-:.\r\n"
    			+ "' ..`\\J.-  \"8-`:. . .  __..--\"\" ...-I  \\ `. `::::::::::::::::::::\".\r\n"
    			+ "`/'\\\\88o. ,O \\  `:.--\"\"....:|:::'''`'\\ ='. }-._'::::::::::::::::::|\r\n"
    			+ "8  8|PP|\"(:. \\-\" \"\"`:::::::|:::.((::='/ .\\\"\"-.:_ ':::::::::::''_.'  _..\r\n"
    			+ " 8  8|::/ \\`::Y  _____`:::::|::::.\\\\[ .\\ \"/\"..* *\"-. '''__..--\"\")\\,\"\".-.\\_\r\n"
    			+ "`\\b d/\"\"===\\==V::.--..__`:::|:::::.|,'*.\"\".:.. \"_-.*`.\"\"    _.-\"-\"\"\\? \"_=``.\r\n"
    			+ "\\\\`\".`\"' .: :-.::.        `:|:::.'.'*.' __..--\"\"   `.*`:--\"\".-\"?,  .)=\"\"`\\ \\\\\r\n"
    			+ " `.``...''_/   ``::      _\\\\--.'.'*.'-\"\"   _..-._ _..>.*;-\"\"@_.-/-\" `\\.-\"\"\"-.\\\r\n"
    			+ "   `-::--\"            .-\"@\"}.'.'*.:)     .\"\\` \\ \\`.--'_`-'     `\\. \\-'-\"\"-   `.\r\n"
    			+ "                     <\\  _...'*.'      .' \\.`\\ `\\ \\\\\"\"         `\\ `' ' .-.\\   |\r\n"
    			+ "                     _\\\"\" .---'        -\\. `\\.-\"\"\"-.\\           \\`|    ._)/   '\r\n"
    			+ "                   .\"\\.`-\"\\`.         `\\. \\-'-\"\"-   `.           \\\\  `---\"   /\r\n"
    			+ "                 .' \\.`\\ `\\ \\\\        `\\ `' ' .-.\\   |            `.       _/\r\n"
    			+ "                 -\\. `\\.-\"\"\"-.\\        \\`|    ._)/   '              `-..--\"\r\n"
    			+ "                `\\. \\-'-\"\"-   `.        \\\\  `---\"   /\r\n"
    			+ "                `\\ `' ' .-.\\   |         `.       _/\r\n"
    			+ "                 \\`|    ._)/   '           `-..--\"\r\n"
    			+ "                  \\\\  `---\"   /\r\n"
    			+ "                   `.       _/\r\n"
    			+ "         _ Seal _    `-..--\""
    			+ "			"
    			+ "");
    	System.out.println("-----------------------------------------------------------------------------------");
    	System.out.println("|                                                                                 |");
    	System.out.println("|                                DesperationRover                                 |");
    	System.out.println("|                                                                                 |");
    	System.out.println("-----------------------------------------------------------------------------------");
//    	SynchronousQueue<String> ControlQ = new SynchronousQueue<String>(true);
//    	SynchronousQueue<String> CommandQ = new SynchronousQueue<String>(true);
    	//System.out.println("-----Soze-----" + ControlQ.isEmpty());
    	Thread Command = new Thread(); {
	    	new CommandThread(ControlQ, CommandQ).start();
	    };
	    Command.start();

	    Thread Control= new Thread(); {
	    	new ControlThread(ControlQ, CommandQ).start();
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
//    public static SynchronousQueue<String> getCommandQ() {
//		return CommandQ;
//    }
//    public static SynchronousQueue<String> getControlQ() {
//		return ControlQ;
//    }

    //Map = new HashSet<>();//Buffer

}
