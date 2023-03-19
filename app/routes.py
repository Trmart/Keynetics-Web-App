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
    page = request.args.get('page', 1, type=int)
    jobs = models.PlugJob.query.order_by(models.PlugJob.start_time.desc()).paginate(page=page, per_page=10)
    configs = models.PlugConfig.query.order_by(models.PlugConfig.name)
    return render_template('jobs.html', page='jobs', title='Jobs', configs=configs, jobs=jobs)


@app.route('/configs', methods=['GET', 'POST'])
def configs():
    page = request.args.get('page', 1, type=int)
    form = forms.PlugConfigForm()
    if form.validate_on_submit():
        config = models.PlugConfig(
            name=form.name.data,
            cure_profile=form.cure_profile.data,
            horizontal_offset=form.horizontal_offset.data,
            vertical_offset=form.vertical_offset.data,
            horizontal_gap=form.horizontal_gap.data,
            vertical_gap=form.vertical_gap.data,
            slot_gap=form.slot_gap.data,
            notes=form.notes.data
        )
        db.session.add(config)
        db.session.commit()
        flash(f'Added {config.name}!', 'success')
        return redirect(url_for('configs'))
    configs = models.PlugConfig.query.order_by(models.PlugConfig.name).paginate(page=page, per_page=5)
    return render_template('configs.html', page='configs', title='Configs', form=form, configs=configs)


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
        config.notes = form.notes.data
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
        form.notes.data = config.notes
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
        slot_gap=config.slot_gap,
        notes=config.notes
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
    if models.PlugJob.query.filter_by(config_id=config_id, status=models.StatusEnum.started).first():
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
    if not job.is_active():
        flash(f'Job for {job.config.name} already stopped!', 'danger')
        return redirect(url_for('jobs'))
    job.status = models.StatusEnum.stopped
    job.end_time = datetime.now()
    job.duration = round((job.end_time - job.start_time).total_seconds() / 60, 2)
    db.session.commit()
    flash(f'Stopped job for {job.config.name}!', 'success')
    return redirect(url_for('jobs'))


@app.route('/stop-all-jobs', methods=['GET', 'POST'])
def stop_all_jobs():
    jobs = models.PlugJob.query.all()
    for job in jobs:
        if job.is_active():
            job.status = models.StatusEnum.stopped
            job.end_time = datetime.now()
            job.duration = round((job.end_time - job.start_time).total_seconds() / 60, 2)
    db.session.commit()
    flash(f'Stopped all jobs!', 'success')
    return redirect(url_for('jobs'))


@app.route('/view-job/<int:job_id>', methods=['GET', 'POST'])
def view_job(job_id):
    job = models.PlugJob.query.get(job_id)
    return render_template('view_job.html', page='jobs', title=f'Job {job.id}', job=job)


@app.route('/add-job-notes/<int:job_id>', methods=['GET', 'POST'])
def add_job_notes(job_id):
    job = models.PlugJob.query.get(job_id)
    form = forms.PlugJobForm()
    if form.validate_on_submit():
        job.notes = form.notes.data
        db.session.commit()
        flash(f'Added notes to job {job.id}!', 'success')
        return redirect(url_for('jobs'))
    else:
        form.notes.data = job.notes
    return render_template('add_job_notes.html', page='jobs', title=f'Add notes to job {job.id}', form=form, job=job)


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


@app.route('/config-plot.png')
def config_plot():
    fig = create_config_plot()
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
    fig.set_size_inches(10, 5)
    fig.subplots_adjust(left=0.07, right=0.97, top=0.93, bottom=0.1)
    return fig


def create_status_plot():
    started = models.PlugJob.query.filter_by(status=models.StatusEnum.started).count()
    stopped = models.PlugJob.query.filter_by(status=models.StatusEnum.stopped).count()
    failed = models.PlugJob.query.filter_by(status=models.StatusEnum.failed).count()
    finished = models.PlugJob.query.filter_by(status=models.StatusEnum.finished).count()
    fig = Figure()
    axis = fig.add_subplot(1, 1, 1)
    axis.pie([started, stopped, failed, finished], labels=['Started', 'Stopped', 'Failed', 'Finished'], autopct='%1.1f%%')
    axis.patches[3].set_facecolor('#00FF00')
    axis.patches[2].set_facecolor('#FF0000')
    axis.patches[1].set_facecolor('#FFFF00')
    axis.patches[0].set_facecolor('#0000FF')
    axis.set_title('Status of Jobs')
    fig.set_size_inches(5, 4)
    fig.subplots_adjust(left=0, right=1, top=0.93, bottom=0.1)
    return fig


def create_config_plot():
    config_counts = {}
    for config in models.PlugConfig.query.all():
        config_counts[config.name] = len(models.PlugJob.query.filter_by(config_id=config.id).all())
    fig = Figure()
    axis = fig.add_subplot(1, 1, 1)
    axis.pie(config_counts.values(), labels=config_counts.keys(), autopct='%1.1f%%')
    axis.set_title('Jobs by Configuration')
    fig.set_size_inches(5, 4)
    fig.subplots_adjust(left=0, right=1, top=0.93, bottom=0.1)
    return fig



# @app.route('/test_wave')
# def test_wave():
#     interface.test_wave()
#     return '<h2>Done!<h2>'


# @app.route('/test_pulse')
# def test_pulse():
#     interface.test_pulse()
#     return '<h2>Done!<h2>'
