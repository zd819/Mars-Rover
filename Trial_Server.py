import socket

s = socket.socket()

s.bind(('0.0.0.0', 8090 )) #Binds IP and port number to newly opened 
s.listen(0)
print("Binded and listening")

while True:

    client, addr = s.accept()
    print("Client accepted %f", addr)

    while True:
        content = client.recv(32)

        if len(content) ==0:
           break

        else:
            print(content)
        

    print("Closing connection")
    client.close()
