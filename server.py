from flask import Flask, request
from flask_socketio import SocketIO

app = Flask(__name__)
socketio = SocketIO(app)

# Default HTML content
html_content = "<h1>Waiting for Data...</h1>"

@app.route('/status.html', methods=['GET', 'POST'])
def status_page():
    global html_content
    if request.method == 'POST':
        html_content = request.data.decode('utf-8')  # Update HTML
        print("Updated HTML:", html_content)
        socketio.emit('update_html', {'content': html_content})  # Push update to clients
        return "Updated", 200
    
    # Serve the full HTML page dynamically
    return """\
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>System Status</title>

    <!-- Include Socket.io -->
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>

    <style>
        /* General Page Styling */
        body {{
            font-family: Arial, sans-serif;
            background-color: #f4f4f4;
            color: #333;
            text-align: center;
            margin: 0;
            padding: 0;
            display: flex;
            justify-content: center;
            align-items: center;
            height: 100vh;
        }}

        /* Status Container */
        .status-container {{
            background: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0px 4px 10px rgba(0, 0, 0, 0.1);
            width: 300px;
        }}

        /* Status Text */
        #status {{
            font-size: 24px;
            font-weight: bold;
            padding: 15px;
            border-radius: 8px;
        }}
    </style>

    <script>
        var socket = io.connect('http://' + document.domain + ':8080');
        socket.on('update_html', function(data) {{
            document.getElementById("status").innerHTML = data.content;
        }});
    </script>
</head>
<body>

    <div class="status-container">
        <h2>System Status</h2>
        <div id="status">{}</div>
    </div>

</body>
</html>
    """.format(html_content)

@socketio.on('connect')
def handle_connect():
    print("Client connected!")
    socketio.emit('update_html', {'content': html_content})  # Send latest content

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=8080)