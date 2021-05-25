var connection = new WebSocket('ws://SERVERIP:PORT');

connection.onopen = function () {
    console.log('Connected!');
    connection.send(Message); // Send the 'message' to the server
};

// Log errors
connection.onerror = function (error) {
    console.log('WebSocket Error ' + error);
};

// Log messages from the server
connection.onmessage = function (e) {
    console.log('Server: ' + e.data);
};
