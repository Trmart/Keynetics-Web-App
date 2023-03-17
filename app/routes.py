'''Module for website routing and rendering.

Maps the webpage URLs to specific functions which handle page logic and
rendering.

'''
from flask import render_template, flash, redirect, url_for, request

from datetime import datetime

from app import app, db, models, forms
# from aquarium.src.fredpi.fredpi import interface


@app.route('/', methods=['GET', 'POST'])
def jobs():
    return render_template('jobs.html', title='Jobs', configs=models.PlugConfig.query.all(), jobs=models.PlugJob.query.all())


@app.route('/configs', methods=['GET', 'POST'])
def configs():
    form = forms.PlugConfigForm()
    if form.validate_on_submit():
        config = models.PlugConfig(
            name=form.name.data,
            cure_profile=form.cure_profile.data,
            horizontal_offset=form.horizontal_offset.data,
            vertical_offset=form.vertical_offset.data,
            horizontal_gap=form.horizontal_gap.data,
            vertical_gap=form.vertical_gap.data,
            slot_gap=form.slot_gap.data
        )
        db.session.add(config)
        db.session.commit()
        flash(f'Added {config.name}!', 'success')
        return redirect(url_for('configs'))
    return render_template('configs.html', title='Configs', form=form, configs=models.PlugConfig.query.all())


@app.route('/help')
def help():
    return render_template('help.html', title='Help')


@app.route('/about')
def about():
    return render_template('about.html', title='About')


@app.route('/edit/<int:config_id>', methods=['GET', 'POST'])
def edit_config(config_id):
    config = models.PlugConfig.query.get(config_id)
    form = forms.PlugConfigForm()
    if form.validate_on_submit():
        config.name = form.name.data
        config.cure_profile = form.cure_profile.data
        config.horizontal_offset = form.horizontal_offset.data
        config.vertical_offset = form.vertical_offset.data
        config.horizontal_gap = form.horizontal_gap.data
        config.vertical_gap = form.vertical_gap.data
        config.slot_gap = form.slot_gap.data
        db.session.commit()
        flash(f'Updated {config.name}!', 'success')
        return redirect(url_for('configs'))
    else:
        form.name.data = config.name
        form.cure_profile.data = config.cure_profile
        form.horizontal_offset.data = config.horizontal_offset
        form.vertical_offset.data = config.vertical_offset
        form.horizontal_gap.data = config.horizontal_gap
        form.vertical_gap.data = config.vertical_gap
        form.slot_gap.data = config.slot_gap
    return render_template('edit_config.html', title=f'Edit {config.name}', form=form, config=config)


@app.route('/copy/<int:config_id>', methods=['GET', 'POST'])
def copy_config(config_id):
    config = models.PlugConfig.query.get(config_id)
    new_config = models.PlugConfig(
        name=f'{config.name} (copy)',
        cure_profile=config.cure_profile,
        horizontal_offset=config.horizontal_offset,
        vertical_offset=config.vertical_offset,
        horizontal_gap=config.horizontal_gap,
        vertical_gap=config.vertical_gap,
        slot_gap=config.slot_gap
    )
    db.session.add(new_config)
    db.session.commit()
    flash(f'Added {new_config.name}!', 'success')
    return redirect(url_for('configs'))


@app.route('/delete-config/<int:config_id>', methods=['GET', 'POST'])
def delete_config(config_id):
    config = models.PlugConfig.query.get(config_id)
    db.session.delete(config)
    db.session.commit()
    flash(f'Deleted {config.name}!', 'success')
    return redirect(url_for('configs'))


@app.route('/start-job', methods=['GET', 'POST'])
def start_job():
    config_id = request.form.get('config_select')
    config = models.PlugConfig.query.get(config_id)
    if models.PlugJob.query.filter_by(config_id=config_id, status='Started').first():
        flash(f'Job for {config.name} already started!', 'danger')
        return redirect(url_for('jobs'))
    job = models.PlugJob(config_id=config_id, start_time=datetime.now())
    db.session.add(job)
    db.session.commit()
    flash(f'Started job for {config.name}!', 'success')
    return redirect(url_for('jobs'))


@app.route('/stop-job/<int:job_id>', methods=['GET', 'POST'])
def stop_job(job_id):
    job = models.PlugJob.query.get(job_id)
    if not job.active():
        flash(f'Job for {job.config.name} already stopped!', 'danger')
        return redirect(url_for('jobs'))
    job.status = models.StatusEnum.stopped
    job.end_time = datetime.now()
    job.duration = round((job.end_time - job.start_time).total_seconds() / 60, 2)
    db.session.commit()
    flash(f'Stopped job for {job.config.name}!', 'success')
    return redirect(url_for('jobs'))


@app.route('/view-job/<int:job_id>', methods=['GET', 'POST'])
def view_job(job_id):
    job = models.PlugJob.query.get(job_id)
    return render_template('view_job.html', title=f'Job {job.id}', job=job)


# @app.route('/test_wave')
# def test_wave():
#     interface.test_wave()
#     return '<h2>Done!<h2>'


# @app.route('/test_pulse')
# def test_pulse():
#     interface.test_pulse()
#     return '<h2>Done!<h2>'
