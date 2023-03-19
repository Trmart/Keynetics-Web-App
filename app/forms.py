'''Module defining forms for user input.

Uses `Flask-WTF`_ to define forms for user input. Forms generally follow
a database model. For example, the fields of a user sign up form will
include many of the fields in the model used to store users.

.. _Flask-WTF
    https://flask-wtf.readthedocs.io/en/1.0.x/

'''
from flask_wtf import FlaskForm
from wtforms import (
    SubmitField,
    StringField,
    FloatField,
    TextAreaField
)
from wtforms.validators import (
    DataRequired,
    Length,
    NumberRange,
    ValidationError
)

from app import models


class PlugConfigForm(FlaskForm):
    name = StringField('Profile Name', validators=[DataRequired(), Length(min=1, max=32)])
    cure_profile = StringField('Cure Profile', validators=[DataRequired(), Length(min=0, max=32)])
    horizontal_offset = FloatField('Horizontal Offset', validators=[DataRequired(), NumberRange(min=0, max=99)])
    vertical_offset = FloatField('Vertical Offset', validators=[DataRequired(), NumberRange(min=0, max=99)])
    horizontal_gap = FloatField('Horizontal Gap', validators=[DataRequired(), NumberRange(min=0, max=99)])
    vertical_gap = FloatField('Vertical Gap', validators=[DataRequired(), NumberRange(min=0, max=99)])
    slot_gap = FloatField('Slot Gap', validators=[DataRequired(), NumberRange(min=0, max=99)])
    notes = TextAreaField('Notes', validators=[Length(min=0, max=255)])
    submit = SubmitField('Save')

    def __repr__(cls):
        return f'PlugConfigForm(name={cls.name.data}, cure_profile={cls.cure_profile.data})'

    def validate_name(self, name):
        config = models.PlugConfig.query.filter_by(name=name.data).first()
        if config is not None:
            raise ValidationError(f'The name "{name.data}" is already in use!')


class PlugJobForm(FlaskForm):
    notes = TextAreaField('Note', validators=[DataRequired(), Length(min=1, max=255)])
    submit = SubmitField('Save')

    def __repr__(cls):
        return f'AddJobNoteForm(notes={cls.notes.data})'
