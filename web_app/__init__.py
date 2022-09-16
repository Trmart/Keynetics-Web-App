from flask import Flask
import dotenv

import os


def get_db_uri():
    db_url = os.environ.get('DATABASE_URL').split(':')
    if db_url[0] == 'postgres':
        db_url[0] = 'postgresql'
    return ':'.join(db_url)

dotenv.load_dotenv()
app = Flask(__name__)

# Flask
app.config['SECRET_KEY'] = os.environ.get('FLASK_SECRET_KEY')

# Database
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
app.config['SQLALCHEMY_DATABASE_URI'] = get_db_uri()

# Mail
app.config['MAIL_SERVER'] = 'smtp.gmail.com'
app.config['MAIL_PORT'] = 587
app.config['MAIL_USE_TLS'] = True
app.config['MAIL_USE_SSL'] = False
app.config['MAIL_USERNAME'] = os.environ.get('EMAIL_USERNAME')
app.config['MAIL_PASSWORD'] = os.environ.get('EMAIL_PASSWORD')

from web_app import routes
