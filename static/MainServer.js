import java.io.IOException;
import java.net.ServerSocket;

public class Server {

    public static void main(String[] args) throws IOException {

        try (new WebsocketServer().start()) {
        } catch (IOException e) {
            System.out.println("Could not create WebSocket Server");
            System.exit(-1);
        }

    }

}
