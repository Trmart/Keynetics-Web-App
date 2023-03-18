'''Module defining database tables.

Uses `Flask-SQLAlchemy`_ to define model classes which represent tables
in a database and their relationships.

.. _Flask-SQLAlchemy
    https://flask-sqlalchemy.palletsprojects.com/en/2.x/

'''
from enum import Enum

from app import db


class PlugConfig(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(32), nullable=False, unique=True)
    cure_profile = db.Column(db.String(32), nullable=False)
    horizontal_offset = db.Column(db.Float, nullable=False)
    vertical_offset = db.Column(db.Float, nullable=False)
    horizontal_gap = db.Column(db.Float, nullable=False)
    vertical_gap = db.Column(db.Float, nullable=False)
    slot_gap = db.Column(db.Float, nullable=False)

    def __init__(self, name, cure_profile, horizontal_offset, vertical_offset, horizontal_gap, vertical_gap, slot_gap):
        self.name = name
        self.cure_profile = cure_profile
        self.horizontal_offset = horizontal_offset
        self.vertical_offset = vertical_offset
        self.horizontal_gap = horizontal_gap
        self.vertical_gap = vertical_gap
        self.slot_gap = slot_gap

    def __repr__(self):
        return f'PlugConfig(id={self.id}, name={self.name}, cure_profile={self.cure_profile})'


class StatusEnum(Enum):
    started = 'Started'
    dispensing = 'Dispensing'
    curing = 'Curing'
    stopped = 'Stopped'
    finished = 'Finished'
    failed = 'Failed'


class PlugJob(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    config_id = db.Column(db.Integer, db.ForeignKey('plug_config.id'), nullable=False)
    config = db.relationship('PlugConfig', backref=db.backref('jobs', lazy=True))
    status = db.Column(db.Enum(StatusEnum), nullable=False, default=StatusEnum.started)
    start_time = db.Column(db.DateTime, nullable=True)
    end_time = db.Column(db.DateTime, nullable=True)
    duration = db.Column(db.Float, nullable=True)

    def __init__(self, config_id, start_time):
        self.config_id = config_id
        self.start_time = start_time

    def __repr__(self):
        return f'PlugJob(id={self.id}, config_id={self.config_id}, status={self.status})'

    def active(self):
        return self.status == StatusEnum.started or self.status == StatusEnum.dispensing or self.status == StatusEnum.curing
