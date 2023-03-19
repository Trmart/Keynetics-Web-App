'''Module for creating the database from scratch.

This should only be run once during setup or in case of database
deletion. Run the file itself with `python create_db.py`.

'''
from datetime import datetime, timedelta
import os
import random

from app import app, db, bcrypt, models


def create_db():
    def random_binary_string(n):
            return ''.join(str(random.randint(0, 1)) for _ in range(n))

    with app.app_context():
        db_path = os.path.join('instance', os.environ.get('DATABASE_URL').split('///')[1])
        if os.path.exists(db_path):
            os.remove(db_path)

        db.create_all()

        # User test data
        for i in range(1, 4):
            user = models.User(
                email='user{}@email.com'.format(i),
                password=bcrypt.generate_password_hash('password{}'.format(i)).decode('utf-8')
            )
            db.session.add(user)
        db.session.commit()

        # PlugConfig test data
        for i in range(4, 8):
            plug = models.PlugConfig(
                name='{}-Pin Plug'.format(i),
                cure_profile=random_binary_string(i),
                horizontal_offset=round(random.uniform(0.1, 5), 2),
                vertical_offset=round(random.uniform(0.1, 5), 2),
                horizontal_gap=round(random.uniform(0.1, 5), 2),
                vertical_gap=round(random.uniform(0.1, 5), 2),
                slot_gap=round(random.uniform(0.1, 5), 2)
            )
            db.session.add(plug)
        db.session.commit()

        # PlugJob test data
        started_job = models.PlugJob(
            config_id=random.randint(1, 4),
            start_time=datetime.now() - timedelta(minutes=random.randint(30, 45))
        )
        db.session.add(started_job)
        db.session.commit()

        for i in range(100):
            job = models.PlugJob(
                config_id=random.randint(1, 4),
                start_time=datetime.now() - timedelta(minutes=random.randint(30, 45))
            )
            job.end_time = job.start_time + timedelta(minutes=random.randint(5, 30))
            job.duration = round((job.end_time - job.start_time).total_seconds() / 60, 2)
            status_list = list(models.StatusEnum)
            status_list.remove(models.StatusEnum.started)
            job.status = random.choice(status_list)

            rand = random.random()
            if rand < 0.8:
                job.status = models.StatusEnum.finished
            elif rand < 0.9:
                job.status = models.StatusEnum.failed
            else:
                job.status = models.StatusEnum.stopped

            db.session.add(job)
        db.session.commit()


if __name__ == '__main__':
    create_db()
