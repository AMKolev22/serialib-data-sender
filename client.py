import websocket
import _thread

class WS_Client:
    def __init__(self):
        websocket.enableTrace(True)
        self.ws = None
        self.connect()

    def connect(self):
        self.ws = websocket.WebSocketApp("ws://localhost:8000",
                                    on_open=self.on_open,
                                    on_message=self.on_message,
                                    on_error=self.on_error,
                                    on_close=self.on_close)
    
    def on_message(self, ws, message):
        print(message)

    def on_error(self, ws, error):
        print(error)

    def on_close(self, ws, close_status_code, close_msg):
        print("### closed ###")

    def on_open(self, ws):
        print("Connected to WebSocket server")

    def start(self):
        def run():
            self.ws.run_forever()
        _thread.start_new_thread(run, ())