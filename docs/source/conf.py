import sys
import os

sys.path.insert(0, os.path.abspath('..'))

project = 'synapticon_ros2_control'
myst_heading_anchors = 5
html_theme = 'sphinx_rtd_theme'
html_sidebars = {
    '**': ['globaltoc.html', 'relations.html', 'searchbox.html'],
}

extensions = [
    'breathe',
    'exhale',
]

breathe_projects = { "MyProject": "build/doxygen/xml" }
breathe_default_project = "MyProject"

exhale_args = {
    "containmentFolder": "./api",
    "rootFileName": "index.rst",
    "rootFileTitle": "C++ API Documentation",
    "doxygenStripFromPath": "/home/application/ros2_ws/src/synapticon_ros2_control/src",
    "createTreeView": True,
}

html_theme_options = {
    "navigation_depth": 3,  # Ensures deeper entries show up
    "collapse_navigation": False,  # Expands sidebar items by default
    "titles_only": False,  # Shows the full structure
}
