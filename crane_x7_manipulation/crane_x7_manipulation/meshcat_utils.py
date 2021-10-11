from pydrake.geometry import Meshcat
from pydrake.common import set_log_level
from IPython.display import Javascript


def start_meshcat(open_window=False):
    try:
        meshcat = Meshcat(8080)
    except RuntimeError:
        pass
    else:
        meshcat = Meshcat()
        web_url = meshcat.web_url()
        set_log_level("warn")
        print('Meshcat is now available at ' f'<a href="{web_url}">{web_url}</a>')
    if open_window:
        print(Javascript(f'window.open("{web_url}");'))
