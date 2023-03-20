'''Module for website routing and rendering.

Maps the webpage URLs to specific functions which handle page logic and
rendering.

'''
from flask import render_template, flash, redirect, url_for, Response, request
from flask_login import login_required, login_user, logout_user, current_user
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure

from datetime import datetime
from datetime import timedelta
import io
import random

from app import app, db, bcrypt, models, forms


@app.route('/', methods=['GET', 'POST'])
def jobs():
    if not current_user.is_authenticated:
        form = forms.UserSignInForm()
        if form.validate_on_submit():
            user = models.User.query.filter_by(email=form.email.data).first()
            if user is None or not bcrypt.check_password_hash(user.password, form.password.data):
                flash('Invalid email or password', 'danger')
                return redirect(url_for('jobs'))
            login_user(user)
            return redirect(url_for('jobs'))
        return render_template('base.html', form=form)
    else:
        page = request.args.get('page', 1, type=int)
        jobs = models.PlugJob.query.order_by(models.PlugJob.start_time.desc()).paginate(page=page, per_page=10)
        configs = models.PlugConfig.query.order_by(models.PlugConfig.name)
        return render_template('pages/jobs.html', title='Jobs', page='jobs', configs=configs, jobs=jobs)


@app.route('/job/<int:job_id>', methods=['GET', 'POST'])
@login_required
def view_job(job_id):
    job = models.PlugJob.query.get(job_id)
    return render_template('pages/view_job.html', title=f'Job #{job.id}', page='jobs', job=job, config=job.config)


@app.route('/add-job-notes/<int:job_id>', methods=['GET', 'POST'])
@login_required
def edit_job(job_id):
    job = models.PlugJob.query.get(job_id)
    form = forms.PlugJobForm()
    if form.validate_on_submit():
        job.notes = form.notes.data
        db.session.commit()
        flash(f'Updated {job.id}!', 'success')
        return redirect(url_for('jobs'))
    else:
        form.notes.data = job.notes
    return render_template('pages/edit_job.html', title=f'Edit Job #{job.id}', page='jobs', form=form, job=job, config=job.config)


@app.route('/start-job', methods=['GET', 'POST'])
@login_required
def start_job():
    config_id = request.form.get('config_select')
    config = models.PlugConfig.query.get(config_id)
    job = models.PlugJob.query.filter_by(status=models.StatusEnum.started).first()
    if job:
        flash(f'A job for {job.config.name} is active!', 'danger')
        return redirect(url_for('jobs'))
    job = models.PlugJob(config_id=config_id, start_time=datetime.now())
    db.session.add(job)
    db.session.commit()
    flash(f'Started job for {config.name}!', 'success')
    return redirect(url_for('jobs'))


@app.route('/stop-job/<int:job_id>', methods=['GET', 'POST'])
@login_required
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
@login_required
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


@app.route('/configs', methods=['GET', 'POST'])
@login_required
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
    return render_template('pages/configs.html', title='Configs', page='configs', form=form, configs=configs)


@app.route('/create-config/', methods=['GET', 'POST'])
@login_required
def create_config():
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
    return render_template('pages/create_config.html', title=f'Create Config', page='configs', form=form)


@app.route('/config/<int:config_id>', methods=['GET', 'POST'])
@login_required
def view_config(config_id):
    config = models.PlugConfig.query.get(config_id)
    return render_template('pages/view_config.html', title=f'{config.name}', page='configs', config=config)


@app.route('/edit-config/<int:config_id>', methods=['GET', 'POST'])
@login_required
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
    return render_template('pages/edit_config.html', title=f'Edit {config.name}', page='configs', form=form, config=config)


@app.route('/copy-config/<int:config_id>', methods=['GET', 'POST'])
@login_required
def copy_config(config_id):
    config = models.PlugConfig.query.get(config_id)
    if models.PlugConfig.query.filter_by(name=f'{config.name} (copy)').first():
        flash(f'Copy of {config.name} already exists! Please rename it first.', 'danger')
        return redirect(url_for('configs'))

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
@login_required
def delete_config(config_id):
    config = models.PlugConfig.query.get(config_id)
    db.session.delete(config)
    db.session.commit()
    flash(f'Deleted {config.name}!', 'success')
    return redirect(url_for('configs'))


@app.route('/insights')
@login_required
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

        'stopped_jobs_duration': "{:.2f}".format(float(calc_total_duration(stopped)) / 60),
        'failed_jobs_duration': "{:.2f}".format(float(calc_total_duration(failed)) / 60),
        'finished_jobs_duration': "{:.2f}".format(float(calc_total_duration(finished)) / 60),
        'all_jobs_duration': "{:.2f}".format(float(calc_total_duration(all)) / 60),

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

    return render_template('pages/insights.html', title='Insights', page='insights', analytics=analytics)


@app.route('/account', methods=['GET', 'POST'])
@login_required
def account():
    email_form = forms.UserEmailForm()
    if email_form.validate_on_submit():
        current_user.email = email_form.email.data
        db.session.commit()
        flash('Your email was updated!', 'success')
        return redirect(url_for('account'))
    else:
        email_form.email.data = current_user.email

    password_form = forms.UserPasswordForm()
    if password_form.validate_on_submit():
        current_user.password = bcrypt.generate_password_hash(password_form.password.data).decode('utf-8')
        db.session.commit()
        flash('Your password was updated!', 'success')
        return redirect(url_for('account'))

    return render_template('pages/account.html', title='Account', page='account', email_form=email_form, password_form=password_form)


@app.route('/help')
@login_required
def help():
    return render_template('pages/help.html', title='Help', page='help')


@app.route('/about')
@login_required
def about():
    return render_template('pages/about.html', title='About', page='about')


@app.route('/api/jobs', methods=['GET', 'POST'])
def api_jobs():
    if request.method == 'POST':
        data = request.get_json(force=True)
        job = models.PlugJob.query.filter_by(id=data['id']).first()
        if job:
            job.status = getattr(models.StatusEnum, data['status'])
            if job.status == 'Finished' or job.status == 'Failed' or job.status == 'Stopped':
                job.end_time = datetime.now()
                job.duration = round((job.end_time - job.start_time).total_seconds() / 60, 2)
            db.session.commit()
        return {'response': 200}
    elif request.method == 'GET':
        jobs = models.PlugJob.query.filter_by(status=models.StatusEnum.started).all()
        jobs = [job.to_dict() for job in jobs]
        return {'jobs': jobs}


@app.route('/durations-plot.png')
@login_required
def durations_plot():
    fig = create_durations_plot()
    output = io.BytesIO()
    FigureCanvas(fig).print_png(output)
    return Response(output.getvalue(), mimetype='image/png')


@app.route('/status-plot.png')
@login_required
def status_plot():
    fig = create_status_plot()
    output = io.BytesIO()
    FigureCanvas(fig).print_png(output)
    return Response(output.getvalue(), mimetype='image/png')


@app.route('/config-plot.png')
@login_required
def config_plot():
    fig = create_config_plot()
    output = io.BytesIO()
    FigureCanvas(fig).print_png(output)
    return Response(output.getvalue(), mimetype='image/png')


@app.route('/logout')
@login_required
def logout():
    logout_user()
    flash('You have been signed out.', 'success')
    return redirect(url_for('jobs'))


def create_durations_plot():
    all = models.PlugJob.query.filter(models.PlugJob.duration.isnot(None)).order_by(models.PlugJob.end_time).limit(100).all()
    end_times = [job.end_time.strftime('%H:%M:%S') for job in all]
    durations = [job.duration for job in all]

    fig = Figure()
    axis = fig.add_subplot(1, 1, 1)
    axis.bar(end_times, durations)
    axis.set_title('Duration of Last 100 Completed Jobs')
    axis.set_xlabel('End Time')
    axis.set_ylabel('Duration (min)')
    fig.set_size_inches(10, 7.5)
    plt.setp(axis.get_xticklabels(), rotation=45, horizontalalignment='right')
    # axis.set_xlim([end_times[0], end_times[-1]])
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
