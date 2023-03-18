'''Module for creating the database from scratch.

This should only be run once during setup or in case of database
deletion. Run the file itself with `python create_db.py`.

'''
from datetime import datetime, timedelta
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
        plug2 = models.PlugConfig(
            name='6-Pin Plug',
            cure_profile='01001011012',
            horizontal_offset=0.9,
            vertical_offset=0.37,
            horizontal_gap=1.23,
            vertical_gap=2.24,
            slot_gap=3
        )
        db.session.add(plug2)
        plug3 = models.PlugConfig(
            name='4-Pin Plug',
            cure_profile='010010110123',
            horizontal_offset=0.9,
            vertical_offset=0.37,
            horizontal_gap=0.9,
            vertical_gap=1.9,
            slot_gap=4
        )
        db.session.add(plug3)
        db.session.commit()

        job1 = models.PlugJob(
            config_id=plug1.id,
            start_time=datetime.now().replace(minute=10),
        )
        job1.end_time = job1.start_time + timedelta(minutes=23)
        job1.duration = round((job1.end_time - job1.start_time).total_seconds() / 60, 2)
        job1.status = models.StatusEnum.finished
        db.session.add(job1)
        job2 = models.PlugJob(
            config_id=plug2.id,
            start_time=datetime.now().replace(minute=20),
        )
        job2.end_time = job2.start_time + timedelta(minutes=5)
        job2.duration = round((job2.end_time - job2.start_time).total_seconds() / 60, 2)
        job2.status = models.StatusEnum.finished
        db.session.add(job2)
        job3 = models.PlugJob(
            config_id=plug3.id,
            start_time=datetime.now().replace(minute=2),
        )
        job3.end_time = job3.start_time + timedelta(minutes=7)
        job3.duration = round((job3.end_time - job3.start_time).total_seconds() / 60, 2)
        job3.status = models.StatusEnum.failed
        db.session.add(job3)
        job4 = models.PlugJob(
            config_id=plug1.id,
            start_time=datetime.now().replace(minute=13),
        )
        job4.end_time = job4.start_time + timedelta(minutes=8)
        job4.duration = round((job4.end_time - job4.start_time).total_seconds() / 60, 2)
        job4.status = models.StatusEnum.stopped
        db.session.add(job4)
        job5 = models.PlugJob(
            config_id=plug2.id,
            start_time=datetime.now().replace(minute=23),
        )
        job5.end_time = job5.start_time + timedelta(minutes=10)
        job5.duration = round((job5.end_time - job5.start_time).total_seconds() / 60, 2)
        job5.status = models.StatusEnum.started

        db.session.commit()


if __name__ == '__main__':
    create_db()
