'''Module for website routing and rendering.

Maps the webpage URLs to specific functions which handle page logic and
rendering.

'''
from flask import render_template, flash, redirect, url_for, Response, request
import matplotlib
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure

from datetime import datetime
import io
import random

from app import app, db, models, forms
# from aquarium.src.fredpi.fredpi import interface


@app.route('/', methods=['GET', 'POST'])
def jobs():
    return render_template('jobs.html', page='jobs', title='Jobs', configs=models.PlugConfig.query.all(), jobs=models.PlugJob.query.all())


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
    return render_template('configs.html', page='configs', title='Configs', form=form, configs=models.PlugConfig.query.all())


@app.route('/insights')
def insights():
    def calc_total_duration(jobs):
        return "{:.2f}".format(sum(job.duration for job in jobs if job.duration is not None))

    def calc_median_duration(jobs):
        ended_jobs = [job for job in jobs if job.duration is not None]
        ended_jobs = sorted(ended_jobs, key=lambda job: job.duration)
        return "{:.2f}".format(ended_jobs[len(ended_jobs) // 2].duration if ended_jobs else 0)

    def calc_mean_duration(jobs):
        return "{:.2f}".format(sum(job.duration for job in jobs if job.duration is not None) / len(jobs) if jobs else 0)

    def calc_std_dev_duration(jobs):
        return "{:.2f}".format((sum((job.duration - sum(job.duration for job in jobs if job.duration is not None) / len(jobs)) ** 2 for job in jobs if job.duration is not None) / len(jobs)) ** 0.5 if jobs else 0)

    def calc_min_duration(jobs):
        return "{:.2f}".format(min(job.duration for job in jobs if job.duration is not None) if jobs else 0)

    def calc_max_duration(jobs):
        return "{:.2f}".format(max(job.duration for job in jobs if job.duration is not None) if jobs else 0)

    all = models.PlugJob.query.all()
    started = models.PlugJob.query.filter_by(status=models.StatusEnum.started).all()
    stopped = models.PlugJob.query.filter_by(status=models.StatusEnum.stopped).all()
    failed = models.PlugJob.query.filter_by(status=models.StatusEnum.failed).all()
    finished = models.PlugJob.query.filter_by(status=models.StatusEnum.finished).all()
    analytics = {
        'started_jobs': len(started),
        'stopped_jobs': len(stopped),
        'failed_jobs': len(failed),
        'finished_jobs': len(finished),
        'all_jobs': len(all),

        'started_jobs_rate': 0,
        'stopped_jobs_rate': 0,
        'failed_jobs_rate': 0,
        'finished_jobs_rate': 0,

        'stopped_jobs_duration': calc_total_duration(stopped),
        'failed_jobs_duration': calc_total_duration(failed),
        'finished_jobs_duration': calc_total_duration(finished),
        'all_jobs_duration': calc_total_duration(all),

        'stopped_jobs_median': calc_median_duration(stopped),
        'failed_jobs_median': calc_median_duration(failed),
        'finished_jobs_median': calc_median_duration(finished),
        'all_jobs_median': calc_median_duration(all),

        'stopped_jobs_mean': calc_mean_duration(stopped),
        'failed_jobs_mean': calc_mean_duration(failed),
        'finished_jobs_mean': calc_mean_duration(finished),
        'all_jobs_mean': calc_mean_duration(all),

        'stopped_jobs_std_dev': calc_std_dev_duration(stopped),
        'failed_jobs_std_dev': calc_std_dev_duration(failed),
        'finished_jobs_std_dev': calc_std_dev_duration(finished),
        'all_jobs_std_dev': calc_std_dev_duration(all),

        'stopped_jobs_min': calc_min_duration(stopped),
        'failed_jobs_min': calc_min_duration(failed),
        'finished_jobs_min': calc_min_duration(finished),
        'all_jobs_min': calc_min_duration(all),

        'stopped_jobs_max': calc_max_duration(stopped),
        'failed_jobs_max': calc_max_duration(failed),
        'finished_jobs_max': calc_max_duration(finished),
        'all_jobs_max': calc_max_duration(all),
    }
    analytics['started_jobs_rate'] = "{:.2f}".format(analytics['started_jobs'] / analytics['all_jobs'] * 100)
    analytics['stopped_jobs_rate'] = "{:.2f}".format(analytics['stopped_jobs'] / analytics['all_jobs'] * 100)
    analytics['failed_jobs_rate'] = "{:.2f}".format(analytics['failed_jobs'] / analytics['all_jobs'] * 100)
    analytics['finished_jobs_rate'] = "{:.2f}".format(analytics['finished_jobs'] / analytics['all_jobs'] * 100)
    return render_template('insights.html', page='insights', title='Insights', analytics=analytics)


@app.route('/help')
def help():
    return render_template('help.html', page='help', title='Help')


@app.route('/about')
def about():
    return render_template('about.html', page='about', title='About')


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
    return render_template('edit_config.html', page='configs', title=f'Edit {config.name}', form=form, config=config)


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
    return render_template('view_job.html', page='jobs', title=f'Job {job.id}', job=job)


@app.route('/durations-plot.png')
def durations_plot():
    fig = create_durations_plot()
    output = io.BytesIO()
    FigureCanvas(fig).print_png(output)
    return Response(output.getvalue(), mimetype='image/png')


@app.route('/status-plot.png')
def status_plot():
    fig = create_status_plot()
    output = io.BytesIO()
    FigureCanvas(fig).print_png(output)
    return Response(output.getvalue(), mimetype='image/png')


def create_durations_plot():
    all = models.PlugJob.query.all()
    all = [job for job in all if job.duration is not None]
    job_ids = [job.id for job in all]
    durations = [job.duration for job in all]

    fig = Figure()
    axis = fig.add_subplot(1, 1, 1)
    axis.bar(job_ids, durations)
    axis.set_title('Duration of Completed Jobs')
    axis.set_xlabel('Job ID')
    axis.set_ylabel('Duration (min)')
    axis.get_xaxis().set_major_formatter(matplotlib.ticker.FuncFormatter(lambda x, p: format(int(x), ',')))
    axis.get_xaxis().set_major_locator(matplotlib.ticker.MaxNLocator(integer=True))
    return fig


def create_status_plot():
    started = models.PlugJob.query.filter_by(status=models.StatusEnum.started).count()
    stopped = models.PlugJob.query.filter_by(status=models.StatusEnum.stopped).count()
    failed = models.PlugJob.query.filter_by(status=models.StatusEnum.failed).count()
    finished = models.PlugJob.query.filter_by(status=models.StatusEnum.finished).count()
    fig = Figure()
    axis = fig.add_subplot(1, 1, 1)
    axis.pie([started, stopped, failed, finished], labels=['Started', 'Stopped', 'Failed', 'Finished'], autopct='%1.1f%%')
    axis.set_title('Status of Jobs')
    return fig


# @app.route('/test_wave')
# def test_wave():
#     interface.test_wave()
#     return '<h2>Done!<h2>'


# @app.route('/test_pulse')
# def test_pulse():
#     interface.test_pulse()
#     return '<h2>Done!<h2>'
