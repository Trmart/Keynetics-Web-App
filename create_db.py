'''Module for creating the database from scratch.

This should only be run once during setup or in case of database
deletion. Run the file itself with `python create_db.py`.

'''
from datetime import datetime, timedelta
import os
import random

from app import app, db, models


def create_db():
    with app.app_context():
        db_path = os.path.join('instance', os.environ.get('DATABASE_URL').split('///')[1])
        if os.path.exists(db_path):
            os.remove(db_path)

        db.create_all()

        # PlugConfig test data
        for i in range(5):
            plug = models.PlugConfig(
                name='{}-Pin Plug'.format(i),
                cure_profile='0100101101',
                horizontal_offset=random.uniform(0.1, 5),
                vertical_offset=random.uniform(0.1, 5),
                horizontal_gap=random.uniform(0.1, 5),
                vertical_gap=random.uniform(0.1, 5),
                slot_gap=random.uniform(0.1, 5)
            )
            db.session.add(plug)
        db.session.commit()

        # PlugJob test data
        for i in range(100):
            job = models.PlugJob(
                config_id=random.randint(1, 5),
                start_time=datetime.now() - timedelta(minutes=random.randint(30, 45)),
            )
            job.end_time = job.start_time + timedelta(minutes=random.randint(5, 30))
            job.duration = round((job.end_time - job.start_time).total_seconds() / 60, 2)
            job.status = random.choice(list(models.StatusEnum))
            db.session.add(job)
        db.session.commit()


if __name__ == '__main__':
    create_db()
