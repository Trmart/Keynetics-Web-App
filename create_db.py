'''Module for creating the database from scratch.

This should only be run once during setup or in case of database
deletion. Run the file itself with `python create_db.py`.

'''
import os

from app import app, db, models


def create_db():
    with app.app_context():
        db_path = os.path.join('instance', os.environ.get('DATABASE_URL').split('///')[1])
        if os.path.exists(db_path):
            os.remove(db_path)

        db.create_all()
        plug1 = models.PlugConfig(
            name='8-Pin Plug',
            cure_profile='0100101101',
            horizontal_offset=0.9,
            vertical_offset=0.37,
            horizontal_gap=1.89,
            vertical_gap=2.99,
            slot_gap=2
        )
        db.session.add(plug1)
        db.session.commit()


if __name__ == '__main__':
    create_db()
