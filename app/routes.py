'''Module for website routing and rendering.

Maps the webpage URLs to specific functions which handle page logic and
rendering.

'''
from flask import render_template

from app import app


@app.route('/')
def home():
    return render_template('home.html')
