from flask import Flask
from flask_socketio import SocketIO
import config

app = Flask('displayer_app')
app.config['SECRET_KEY'] = config.secretKey
socketio = SocketIO(app, async_mode='gevent')
