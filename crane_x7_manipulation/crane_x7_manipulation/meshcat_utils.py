from pydrake.common import set_log_level
from pydrake.geometry import Meshcat

from IPython.display import display, HTML, Javascript



def start_meshcat(open_window=False):
    meshcat = Meshcat()
    web_url = meshcat.web_url()

    set_log_level("warn")
    display(
        HTML('Meshcat is now available at '
             f'<a href="{web_url}">{web_url}</a>'))

    if open_window:
        display(Javascript(f'window.open("{web_url}");'))

    return meshcat