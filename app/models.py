'''Module defining database tables.

Uses `Flask-SQLAlchemy`_ to define model classes which represent tables
in a database and their relationships.

.. _Flask-SQLAlchemy
    https://flask-sqlalchemy.palletsprojects.com/en/2.x/

'''
from flask_login import UserMixin

from enum import Enum

from app import db, login_manager


@login_manager.user_loader
def load_user(user_id):
    return User.query.get(int(user_id))


class User(db.Model, UserMixin):
    id = db.Column(db.Integer, primary_key=True)
    email = db.Column(db.String(64), nullable=False, unique=True)
    password = db.Column(db.String(32), nullable=False)

    def __init__(self, email, password):
        self.email = email
        self.password = password

    def __repr__(self):
        return f'User(id={self.id}, email={self.email})'

    def to_dict(self):
        return {
            'id': self.id,
            'email': self.email
        }


class PlugConfig(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(32), nullable=False, unique=True)
    cure_profile = db.Column(db.String(32), nullable=False)
    horizontal_offset = db.Column(db.Float, nullable=False)
    vertical_offset = db.Column(db.Float, nullable=False)
    horizontal_gap = db.Column(db.Float, nullable=False)
    vertical_gap = db.Column(db.Float, nullable=False)
    slot_gap = db.Column(db.Float, nullable=False)
    notes = db.Column(db.String(256), nullable=True)

    def __init__(self, name, cure_profile, horizontal_offset, vertical_offset, horizontal_gap, vertical_gap, slot_gap, notes=''):
        self.name = name
        self.cure_profile = cure_profile
        self.horizontal_offset = horizontal_offset
        self.vertical_offset = vertical_offset
        self.horizontal_gap = horizontal_gap
        self.vertical_gap = vertical_gap
        self.slot_gap = slot_gap
        self.notes = notes

    def __repr__(self):
        return f'PlugConfig(id={self.id}, name={self.name}, cure_profile={self.cure_profile})'

    def to_dict(self):
        return {
            'id': self.id,
            'name': self.name,
            'cure_profile': self.cure_profile,
            'horizontal_offset': self.horizontal_offset,
            'vertical_offset': self.vertical_offset,
            'horizontal_gap': self.horizontal_gap,
            'vertical_gap': self.vertical_gap,
            'slot_gap': self.slot_gap,
            'notes': self.notes
        }


class StatusEnum(Enum):
    started = 'started'
    stopped = 'stopped'
    finished = 'finished'
    failed = 'failed'


class PlugJob(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    config_id = db.Column(db.Integer, db.ForeignKey('plug_config.id'), nullable=False)
    config = db.relationship('PlugConfig', backref=db.backref('jobs', lazy=True))
    status = db.Column(db.Enum(StatusEnum), nullable=False, default=StatusEnum.started)
    start_time = db.Column(db.DateTime, nullable=True)
    end_time = db.Column(db.DateTime, nullable=True)
    duration = db.Column(db.Float, nullable=True)
    notes = db.Column(db.String(256), nullable=True)

    def __init__(self, config_id, start_time, notes=''):
        self.config_id = config_id
        self.start_time = start_time
        self.notes = notes

    def __repr__(self):
        return f'PlugJob(id={self.id}, config_id={self.config_id}, status={self.status})'

    def is_active(self):
        return self.status == StatusEnum.started

    def to_dict(self):
        return {
            'id': self.id,
            'config_id': self.config_id,
            'status': self.status.value,
            'start_time': self.start_time.timestamp() if self.start_time else None,
            'end_time': self.end_time,
            'duration': self.duration,
            'notes': self.notes,
            'config': self.config.to_dict()
        }
