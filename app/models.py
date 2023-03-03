'''Module defining database tables.

Uses `Flask-SQLAlchemy`_ to define model classes which represent tables
in a database and their relationships.

.. _Flask-SQLAlchemy
    https://flask-sqlalchemy.palletsprojects.com/en/2.x/

'''
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
