import requests

import dotenv
import os


dotenv.load_dotenv()
APP_URL = os.environ.get('APP_URL')


def test_api_get():
    response = requests.get(APP_URL + 'api').json()
    assert len(response['jobs']) == 0 or len(response['jobs']) == 1
    if len(response['jobs']) == 1:
        assert response['jobs'][0]['id'] == 1
        assert response['jobs'][0]['status'] == 'started'


def test_api_post():
    status_update = {
        'id': 1,
        'status': 'finished',
    }
    requests.post(APP_URL + 'api', json=status_update)
    response = requests.get(APP_URL + 'api').json()
    assert len(response['jobs']) == 0 or len(response['jobs']) == 1
    if len(response['jobs']) == 1:
        assert response['jobs'][0]['id'] == 1
        assert response['jobs'][0]['status'] == 'finished'


if __name__ == '__main__':
    test_api_get()
    test_api_post()
