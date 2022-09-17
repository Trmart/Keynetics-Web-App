'''Provides the interface to the Flask app.

Creates the Flask app instance so `run.py` can run it. Also defines the
app configuration.

'''
from flask import Flask


app = Flask(__name__)


from app import routes
