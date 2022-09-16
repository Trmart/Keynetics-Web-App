#holds the configuration. makes a module. sets up other modules used in app. 
from flask import Flask

app = Flask(__name__)




from web_app import routes