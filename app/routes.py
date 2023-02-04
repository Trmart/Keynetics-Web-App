'''Module for website routing and rendering.

Maps the webpage URLs to specific functions which handle page logic and
rendering.

'''
from flask import render_template

from app import app

from aquarium.src.fredpi.fredpi import interface


@app.route('/')
def home():
    return render_template('home.html', title='Home')

@app.route('/about')
def about():
    return render_template('about.html', title='About')

@app.route('/codingprofiles')
def codingprofiles():
    return render_template('codingprofiles.html', title='Coding Profiles')

@app.route('/jobs')
def jobs():
    return render_template('jobs.html', title='Jobs')

@app.route('/test_wave')
def test_wave():
    interface.test_wave()
    return '<h2>Done!<h2>'
