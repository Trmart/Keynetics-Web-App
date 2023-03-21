'''Module to start the Flask server running.

Simply calls run on the Flask app instance. Run the file itself with
`python run.py`.

'''
from app import app


if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
