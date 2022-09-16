from flask import render_template
from flask_login import current_user

from web_app import app


@app.route('/')
def home():
    return render_template('home.html')


@app.route('/sign-up', methods=['GET', 'POST'])
def sign_up():
    pass


@app.route('/sign-in', methods=['GET', 'POST'])
def sign_in():
    pass
