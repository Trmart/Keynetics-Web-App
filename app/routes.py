'''Module for website routing and rendering.

Maps the webpage URLs to specific functions which handle page logic and
rendering.

'''
from flask import render_template, flash, redirect, url_for

from app import app, db, models, forms

from aquarium.src.fredpi.fredpi import interface


@app.route('/')
def home():
    return render_template('home.html', title='Home')


@app.route('/about')
def about():
    return render_template('about.html', title='About')


@app.route('/configs', methods=['GET', 'POST'])
def configs():
    form = forms.PlugConfigForm()
    if form.validate_on_submit():
        plug_config = models.PlugConfig(
            name=form.name.data,
            cure_profile=form.cure_profile.data,
            horizontal_offset=form.horizontal_offset.data,
            vertical_offset=form.vertical_offset.data,
            horizontal_gap=form.horizontal_gap.data,
            vertical_gap=form.vertical_gap.data,
            slot_gap=form.slot_gap.data
        )
        db.session.add(plug_config)
        db.session.commit()
        flash(f'Added {plug_config.name}!', 'success')
        return redirect(url_for('home'))
    return render_template('configs.html', title='Configs', form=form, configs=models.PlugConfig.query.all())


@app.route('/jobs', methods=['GET', 'POST'])
def jobs():
    return render_template('jobs.html', title='Jobs')


@app.route('/help')
def help():
    return render_template('help.html', title='Help')


@app.route('/delete-config/<int:config_id>')
def delete_config(config_id):
    print(config_id)
    plug_config = models.PlugConfig.query.get(config_id)
    db.session.delete(plug_config)
    db.session.commit()
    flash(f'Deleted {plug_config.name}!', 'success')
    return redirect(url_for('configs'))


@app.route('/test_wave')
def test_wave():
    interface.test_wave()
    return '<h2>Done!<h2>'


@app.route('/test_pulse')
def test_pulse():
    interface.test_pulse()
    return '<h2>Done!<h2>'
