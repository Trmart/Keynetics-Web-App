'''Provides the interface to the Flask app.

Creates the Flask app instance so `run.py` can run it. Also defines the
app configuration.

'''
import dotenv
from flask import Flask
from flask_sqlalchemy import SQLAlchemy
from flask_login import LoginManager
from flask_bcrypt import Bcrypt

import os


dotenv.load_dotenv()
app = Flask(__name__)
app.config['SECRET_KEY'] = os.environ.get('FLASK_SECRET_KEY')
app.config['SQLALCHEMY_DATABASE_URI'] = os.environ.get('DATABASE_URL')
db = SQLAlchemy(app)
login_manager = LoginManager(app)
bcrypt = Bcrypt(app)


from app import routes
