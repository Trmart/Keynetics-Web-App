from flask import render_template

from web_app import app


@app.route('/')
def home():
    return render_template('home.html')


@app.route('/sign-up', methods=['GET', 'POST'])
def sign_up():
    return "text"


@app.route('/sign-in', methods=['GET', 'POST'])
def sign_in():
    pass
